(ns idp.robot.brain.travel
  (:require
    [taoensso.encore :as enc]
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
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
  :chain
  [[junction-approach-turn {:turn-direction :right
                            :turn-forward-speed -150
                            :excess-coeff 1.6}]
   [follow-up-to-blackout]]
  :tick (fn [robot] (phase/tick-chain robot)))

(defphase post-tunnel-collect-turn
  :init {:overshoot-status :finding}
  :sub-phases
  {:us-turning-condition
   [tracking-prolonged-condition
    {:min-duration 80
     :pred (fn [{:keys [readings]}]
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
    (let [cmd (phase/tick-subphase robot :find-junction)]
      (if (phase/phase-done? cmd :find-junction)
        (phase/mark-done cmd)
        cmd))))

(defphase post-tunnel-to-collect
  :chain (fn [{:keys [collection-target]}]
           [[post-tunnel-collect-turn]
            [up-to-collect {:collection-target collection-target}]])
  :tick
  (fn [robot] (phase/tick-chain robot)))

;; Home

(defphase finalise-home
  :chain [[timed-straight {:duration 4000 :speed 255}]
          [timed-straight {:duration 1700 :speed -100}]]
  :tick (fn [robot] (phase/tick-chain robot)))

#_(defphase align-to-home
  "Takes a robot on the home path up to the edge of the home box,
  aligning it so it is as straight as possible"
  :init {:sub-status :find-edge}
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :find-edge
      (let [found-edge?
            (match (get-combined-line-readings readings)
              [_ :w :w :w] true
              [:w :w :w _] true
              :else false)])
      )))

(defphase home-follow
  :sub-phases
  {:follow [basic-follow]}
  :tick
  (fn [{:keys [readings] :as robot}]
    (let [cmd (phase/tick-subphase robot :follow)
          done? (match (get-combined-line-readings readings)
                  [:w :w :w _] true
                  [_ :w :w :w] true
                  :else false)]
      (if done?
        (phase/mark-done cmd)
        cmd))))

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
                                :excess-coeff 1.6
                                :turn-forward-speed -150}]
       [home-follow]]))
  :tick
  (fn [robot] (phase/tick-chain robot)))


#_(defphase centre-block-180
  :init {:nturns 0}
  :sub-phases
  {:turn1 [timed-straight nil {:duration 2000
                               :speed 0
                               :turn-speed 200}]
   :turn2 [junction-turn-spin nil {:turn-direction :right}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (let [nturns (:nturns state)
          first? (= 0 nturns)
          ss (if first? :turn1 :turn2)
          cmd (if first?
                (phase/tick-subphase robot :turn1)
                (phase/tick-subphase robot :turn2))]
      (if (phase/phase-done? cmd ss)
        (if first?
          (update cmd :state assoc :nturns (inc nturns))
          (phase/mark-done cmd))
        cmd))))