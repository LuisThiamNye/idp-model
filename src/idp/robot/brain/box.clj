(ns idp.robot.brain.box
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [idp.robot.brain.util :as bu]
    [chic.util.ns :refer [inherit-vars]]
    [clojure.core.match :refer [match]]
    [idp.robot.brain.follow :as follow]
    [idp.robot.state :as rs]
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
  bu/set-input
  bu/tracking-motor-forward
  bu/straight-up-to-blackout
  bu/tracking-motor-forward
  junction/up-to-junction
  follow/biased-follow
  follow/basic-follow)

(defn reattempt-collection?
  "After dropping off block, decides whether to try collecting another
  block or to go back home"
  [{:keys [competition-start-time collection-target force-home?] :as _merged-state}]
  (if force-home?
    (do (println "Not reattemting due to force option")
      false)
    (and collection-target
      (let [min-collect-duration (* 1 60 1000)
            competition-duration (* 5 60 1000) ;; 5 mins
            time-passed (- (System/currentTimeMillis) competition-start-time)
            time-remaining (- competition-duration time-passed)
            result (<= min-collect-duration time-remaining)]
        (println "Reattempt: " result
          " (remaining: " (long (/ time-remaining 1000)) " s)")
        result))))

(defphase decide-next-target
  "After having dropped off a block, decides where the robot should go next"
  :tick
  (fn [{:keys [merged-state]}]
    (phase/mark-done
      {:output {:next-target
                {:go-home? (not (reattempt-collection? merged-state))
                 :collection-target
                 (case (:collection-target merged-state)
                   2 1
                   ;; only go for two blocks
                   ; 1 3
                   1 nil
                   3 nil)}}})))

(defphase backup-from-box-turn
  :init {}
  :sub-phases
  (fn [{:keys [go-home? density]}]
    {:turn [junction-turn-spin {:turn-direction
                                (if (enc/have boolean? go-home?)
                                  (case density
                                    :high :right
                                    :low :left)
                                  :right)}]})
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/tick-subphase robot :turn)]
      (if (phase/phase-done? cmd :turn)
        (phase/mark-done cmd)
        cmd))))

(defphase backup-from-box-blackout-retreat
  :tick
  (fn [{:keys [readings]}]
    (let [ls (get-combined-line-readings readings)]
      (if (match ls
            [_ :w _ _] true
            [_ _ :w _] true
            :else false)
        {:input (motor-input -255)}
        (phase/mark-done {})))))

(defphase backup-from-box
  "Reverse from dropping off block at box"
  :init {:status :blackout-retreating}
  :chain
  [[backup-from-box-blackout-retreat]
   [timed-straight {:duration 1100
                    :speed -255}]]
  :tick
  (fn [robot] (phase/tick-chain robot)))

(defphase box-approach-edge
  "Go up to edge of box"
  :init {:status :up-to-line}
  :chain [[set-input {:grabber-position :open}]
          [straight-up-to-blackout]
          [timed-straight {:duration 300}]]
  :tick
  (fn [robot] (phase/tick-chain robot)))

(defphase tunnel-dropoff-overshoot
  :chain [[timed-straight {:duration 1600 :speed 180}]
          [timed-straight {:duration 2600
                           :speed 0 :turn-speed -150}]]
  :tick
  (fn [robot] (phase/tick-chain robot)))

(defphase up-to-dropoff-box
  "Goes from tunnel up to one of the three junctions of a line box.
  Performs right-biased line following to ensure junction detection."
  :init {:overshoot-status :finding}
  :sub-phases
  (fn [{:keys [density]}]
    {:us-turning-condition
     [tracking-prolonged-condition
      {:min-duration 80
       :pred (fn [{:keys [readings]}]
               (<= 200 (rs/get-rear-ultrasonic readings) 500))}]
     :overshoot [tunnel-dropoff-overshoot]
     :find-box [up-to-junction
                {:junction-number (case density
                                    :high 3 ;; red box
                                    :low 1 ;; green box
                                    (throw (ex-info "No density!" {})))
                 :turn-direction :right}]})
  :tick
  (fn [{{:keys [overshoot-status]} :state :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :find-box)
                (when (= :finding overshoot-status)
                  (phase/merge-cmds
                    {:input {:ultrasonic-active? true}}
                    (phase/tick-subphase robot :us-turning-condition))))]
      (cond
        (phase/phase-done? cmd :find-box)
        (phase/mark-done cmd)
        
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