(ns idp.robot.brain.travel
  "Miscellaneous phases"
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [clojure.core.match :refer [match]]
    [chic.util.ns :refer [inherit-vars]]
    [idp.robot.state :as rs]
    [idp.robot.brain.util :as bu]
    [idp.robot.brain.follow :as follow]
    [idp.robot.brain.junction :as junction]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input
  bu/get-combined-line-readings
  bu/tracking-prolonged-condition
  bu/until-straight
  bu/until-turning
  bu/junction-turn-spin
  bu/straight-up-to-blackout
  bu/position-grabber
  bu/tracking-motor-forward
  junction/junction-approach-turn
  junction/up-to-junction
  follow/biased-follow
  follow/basic-follow
  follow/follow-up-to-blackout)

;; Collecting 1 & 3

(defphase branch-to-main
  "Turns from collecting a block at a branch back onto the main line"
  :init {}
  :sub-phases
  {:turn [junction-turn-spin {:turn-direction :right}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/tick-subphase robot :turn)]
      (if (phase/phase-done? cmd :turn)
        (phase/mark-done cmd)
        cmd))))

(defphase collect-from-junction
  "Turns the robot from the main line and drives it up to the end
  of the branch line where the block is."
  :chain
  [[junction-approach-turn {:turn-direction :right
                            :turn-forward-speed -150
                            :excess-coeff 1.6}]
   [follow-up-to-blackout]]
  :tick (fn [robot] (phase/tick-chain robot)))

(defphase post-tunnel-collect-turn
  "Handles the curved line after the tunnel, when approaching the collection point.
  By overshooting, this helps attain a better angle that makes detecting junctions
  more reliable."
  :init {:overshoot-status :finding}
  :sub-phases
  {:us-turning-condition
   [tracking-prolonged-condition
    {:min-duration 80
     :pred (fn [{:keys [readings]}]
             ;; this condition implies we have started turning at the curved line
             (<= 200 (rs/get-rear-ultrasonic readings) 500))}]
   :overshoot [timed-straight {:duration 2000 :speed 180}]
   :follow [biased-follow {:bias :right}]}
  :tick
  (fn [{{:keys [overshoot-status]} :state
        :keys [readings] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :us-turning-condition)
                (phase/tick-subphase robot :follow)
                {:input {:ultrasonic-active? true}})]
      (cond
        (and (= :done overshoot-status)
          (match (get-combined-line-readings readings)
            [:w _  _ _] true
            [_ :w  _ _] true
            :else false))
        (phase/mark-done
          (phase/merge-cmds cmd
            {:input {:ultrasonic-active? false}}))
        
        (or (= overshoot-status :doing)
          (and (= overshoot-status :finding)
            (phase/phase-done? cmd :us-turning-condition)))
        (let [cmd (phase/merge-cmds cmd
                    {:status {:overshoot-status :doing}}
                    (phase/tick-subphase robot :overshoot))]
          (if (phase/phase-done? cmd :overshoot)
            (phase/merge-cmds cmd
              {:state {:overshoot-status :done}
               :input {:ultrasonic-active? false}})
            cmd))
        
        :else
        cmd))))

(defphase up-to-collect
  "Goes from tunnel on the collect-side main line up to one of the
  junctions to collect a block.
  Performs biased line following to ensure junction detection."
  :sub-phases
  (fn [{:keys [collection-target]}]
    {:find-junction [up-to-junction
                     {:junction-number (enc/have #{1 3} collection-target)
                      :turn-direction :right}]})
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :find-junction)
                {:input {:grabber-position :open}})]
      (if (phase/phase-done? cmd :find-junction)
        (phase/mark-done cmd)
        cmd))))

(defphase post-tunnel-to-collect
  "Brings the robot from the end of the tunnel up to the junction
  of the next block to be collected."
  :chain (fn [{:keys [collection-target]}]
           [[post-tunnel-collect-turn]
            [up-to-collect {:collection-target collection-target}]])
  :tick
  (fn [robot] (phase/tick-chain robot)))

;; Home

(defphase finalise-home
  "Final manoeuvre that drives the robot from the edge of the home box
  and positions it within the bounds of the box.
  Involves a collision with the wall, followed by a small retreat, to
  decrease error."
  :chain [[timed-straight {:duration 4000 :speed 255}]
          [timed-straight {:duration 1700 :speed -100}]]
  :tick (fn [robot] (phase/tick-chain robot)))

(defphase home-follow
  "Does line following along the path up to the edge of the home box"
  :init {:status :retreating}
  :sub-phases
  {:retreat [timed-straight {:duration 200 :speed -255}]
   :follow [basic-follow]}
  :tick
  (fn [{:keys [readings state] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :follow)
                (when (= :retreating (:status state))
                  (phase/tick-subphase robot :retreat)))
          done? (and (= :following (:status state))
                  (match (get-combined-line-readings readings)
                    [:w :w :w _] true
                    [_ :w :w :w] true
                    :else false))]
      (if done?
        (phase/mark-done cmd)
        (if (and (= :retreating (:status state))
              (phase/phase-done? cmd :retreat))
          (phase/merge-cmds cmd
            {:state {:status :following}})
          cmd)))))

(defphase up-to-home-entry
  "Drives robot from a collection point up to the home junction,
  then spins it pointing towards the home box"
  :chain
  (fn [{:keys [density]}]
    (let [turn-direction (case density
                           :high :left
                           :low :right)]
      [[up-to-junction {:turn-direction turn-direction
                        :junction-number 1}]
       [junction-approach-turn {:turn-direction turn-direction
                                :excess-coeff 1.35
                                :turn-forward-speed -170}]
       [home-follow]]))
  :tick
  (fn [robot] (phase/tick-chain robot)))
