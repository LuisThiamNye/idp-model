(ns idp.robot.brain.ramp
  "Phases relating to transporting the robot across the ramp"
  (:require
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [clojure.core.match :refer [match]]
    [chic.util.ns :refer [inherit-vars]]
    [idp.robot.state :as rs]
    [idp.robot.brain.util :as bu]
    [idp.robot.brain.follow :as follow]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input
  bu/get-combined-line-readings
  bu/tracking-prolonged-condition
  bu/until-straight
  bu/until-turning
  bu/straight-up-to-blackout
  follow/biased-follow
  follow/basic-follow
  follow/follow-correcting)

(defphase post-ramp-to-centre-block
  "Does right-biased line following up to the central block collection point."
  :init
  {:rhs-lines-found 0
   :lhs-lines-found 0}
  :sub-phases
  {:follow [follow-correcting {:phase-decl [biased-follow {:bias :right}]}]}
  :tick
  (fn [{:keys [state readings]
        {:keys [rhs-lines-found lhs-lines-found]} :state
        :as robot}]
    (let [combined-readings (get-combined-line-readings readings)
          [lhs-lines-found rhs-lines-found]
          (if (= combined-readings (:combined-line-readings state))
            [lhs-lines-found rhs-lines-found]
            (match combined-readings
              [:w :w :w :w]
              [(inc lhs-lines-found) (inc rhs-lines-found)]
              [:b :w :w :w]
              [lhs-lines-found (inc rhs-lines-found)]
              [:w :w :w :b]
              [(inc lhs-lines-found) rhs-lines-found]
              :else
              [lhs-lines-found rhs-lines-found]))
          done? (and
                  (or (<= 1 rhs-lines-found)
                    (<= 1 lhs-lines-found))
                  (:block-present? readings))
          cmd (phase/tick-subphase robot :follow)]
      (if done?
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          {:state {:lhs-lines-found lhs-lines-found
                   :rhs-lines-found rhs-lines-found
                   :combined-line-readings combined-readings}})))))

;; Leave ramp

(defphase post-ramp-straighten
  "Completes once the robot is going approximately straight
  after tackling the turning after the ramp."
  :init {}
  :sub-phases
  {:until-straight [until-straight {:min-straight-duration 500
                                    :max-turn-rate 60}]
   :us-condition
   [tracking-prolonged-condition
    {:min-duration 1200
     :pred (fn [{:keys [readings]}]
             ;; require that we be a minimum distance from the wall behind the robot
             (< 300 (rs/get-rear-ultrasonic readings) 800))}]
   :follow [follow-correcting {:phase-decl [basic-follow]}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :until-straight)
                (phase/tick-subphase robot :follow)
                (phase/tick-subphase robot :us-condition)
                {:input {:ultrasonic-active? true}})]
      (if (and (phase/phase-done? cmd :until-straight)
            (phase/phase-done? cmd :us-condition))
        (phase/mark-done
          (phase/merge-cmds cmd
            {:input {:ultrasonic-active? false}}))
        cmd))))

(defphase post-ramp-find-junction
  "Completes once the robot has started turning at the turning
  after the ramp."
  :init {}
  :sub-phases
  {:us-turning-condition
   [tracking-prolonged-condition
    {:min-duration 80
     :pred (fn [{:keys [readings]}]
             ;; once turning, we should start seeing the wall behind us
             (<= 140 (:ultrasonic-2 readings) 1000))}]
   :until-turning [until-turning {:turn-direction :left
                                  :min-turn-rate 200}]
   :follow [follow-correcting {:phase-decl [basic-follow {:low-power? false}]}]}
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :follow)
                (phase/tick-subphase robot :us-turning-condition)
                (phase/tick-subphase robot :until-turning)
                {:input {:ultrasonic-active? true}})]
      (if (or (phase/phase-done? cmd :us-turning-condition)
            (phase/phase-done? cmd :until-turning))
        (phase/mark-done
          (phase/merge-cmds cmd
            {:state {:status :straightening}
             :input {:ultrasonic-active? false}}))
        cmd))))

(defphase leave-ramp
  "Brings robot from on top of the ramp, up to after the turning"
  :init {}
  :chain [[post-ramp-find-junction]
          [post-ramp-straighten]]
  :tick
  (fn [{:keys [] :as robot}]
    (phase/tick-chain robot)))

;; Up to ramp

(defphase up-to-ramp-find-turn
  "Completes once the turning prior to the ramp has been detected"
  :init {}
  :sub-phases
  {:follow [follow-correcting {:phase-decl [biased-follow {:bias :left}]}]
   :turn-us-condition [tracking-prolonged-condition
                       {:min-duration 400
                        :pred (fn [{:keys [readings]}]
                                ;; rear distance from the wall should be small enough
                                (<= 500 (:ultrasonic-2 readings) 800))}]}
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :follow)
                (phase/tick-subphase robot :turn-us-condition)
                {:input {:ultrasonic-active? true}})]
      (if (phase/phase-done? robot :turn-us-condition)
        (phase/mark-done
          (phase/merge-cmds cmd
            {:input {:ultrasonic-active? false}}))
        cmd))))

(defphase up-to-ramp-straighten
  "Completes once the robot is going approximately straight
  on top of the ramp structure."
  :init {}
  :sub-phases
  {:follow [basic-follow nil {:high-power? true}]
   :us-condition
   [tracking-prolonged-condition
    {:min-duration 1500
     :pred (fn [{:keys [readings]}]
             ;; rear ultrasonic sensor should not see any close objects
             ;; if the robot is level and on top of the ramp
             (not (< 0 (rs/get-rear-ultrasonic readings) 1000)))}]
   :until-straight [until-straight {:min-straight-duration 2500
                                    :max-turn-rate 15}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :until-straight)
                (phase/tick-subphase robot :us-condition)
                (phase/tick-subphase robot :follow)
                {:input {:ultrasonic-active? true}})]
      (if (and (phase/phase-done? cmd :us-condition)
            (phase/phase-done? cmd :until-straight))
        (phase/mark-done
          (phase/merge-cmds cmd
            {:input {:ultrasonic-active? false}}))
        cmd))))

(defphase up-to-ramp
  "Drives robot (pointed right) from home side of the table
  up to the ramp, then travelling a significant straight distance across it"
  :init {}
  :chain [[up-to-ramp-find-turn]
          [up-to-ramp-straighten]]
  :tick
  (fn [robot] (phase/tick-chain robot)))

(defphase cross-from-start-by-ramp
  :chain [[up-to-ramp]
          [leave-ramp]]
  :tick (fn [robot] (phase/tick-chain robot)))