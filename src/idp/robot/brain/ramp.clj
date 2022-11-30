(ns idp.robot.brain.ramp
  (:require
    [taoensso.encore :as enc]
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
  follow/basic-follow)

(defphase post-ramp-to-centre-block
  "Does right-biased line following up to central collection point."
  :init
  {:rhs-lines-found 0
   :lhs-lines-found 0}
  :sub-phases
  {:follow [biased-follow {:bias :right}]}
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
                  (<= 1 rhs-lines-found)
                  (<= 1 lhs-lines-found)
                  (:block-present? readings))
          cmd (phase/tick-subphase robot :follow)]
      (if done?
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          {:state {:lhs-lines-found lhs-lines-found
                   :rhs-lines-found rhs-lines-found
                   :combined-line-readings combined-readings}})))))

;; Leave ramp

#_(defphase post-ramp-turning
  :init {}
  :sub-phases
  {:turn [tracking-motor-turn]}
  :tick
  (fn [{:as robot}]
    (let [;; after this turn amount, we have turned enough
          turn-amount-thres (long 3e5)
          turn-cmd (phase/tick-subphase robot :turn)
          done? (<= (-> turn-cmd :state :sub-phases :turn :turn-amount)
                  (- turn-amount-thres))]
      (if done?
        (phase/mark-done turn-cmd)
        turn-cmd))))

(defphase post-ramp-straighten
  :init {}
  :sub-phases
  {:until-straight [until-straight {:min-straight-duration 400
                                    :max-turn-rate 60}]
   :follow [biased-follow {:bias :left}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :until-straight)
                (phase/tick-subphase robot :follow))]
      (if (phase/phase-done? cmd :until-straight)
        (phase/mark-done cmd)
        cmd))))

(defphase post-ramp-find-junction
  :init {}
  :sub-phases
  {:us-turning-condition [tracking-prolonged-condition
                          {:min-duration 80
                           :pred (fn [{:keys [readings]}]
                                   (<= 140 (:ultrasonic-2 readings) 500))}]
   :follow [biased-follow {:bias :left}]}
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :follow)
                (phase/tick-subphase robot :us-turning-condition)
                {:input {:ultrasonic-active? true}})]
      (if (phase/phase-done? cmd :us-turning-condition)
        (phase/mark-done
          (phase/merge-cmds cmd
            {:state {:status :straightening
                     :sub-phases {:follow {:high-power? false}}}
             :input {:ultrasonic-active? false}}))
        cmd))))

(defphase leave-ramp
  :init {}
  :chain [[post-ramp-find-junction]
          [post-ramp-straighten]]
  :tick
  (fn [{:keys [] :as robot}]
    (phase/tick-chain robot)))

;; Up to ramp

(defphase up-to-ramp-find-turn
  :init {}
  :sub-phases
  {:follow [biased-follow {:bias :left}]
   :turn-us-condition [tracking-prolonged-condition
                       {:min-duration 400
                        :pred (fn [{:keys [readings]}]
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
  :init {}
  :sub-phases
  {:follow [basic-follow nil {:high-power? true}]
   ; :until-turn [until-turning {:turn-direction :left}]
   :until-straight [until-straight {:min-straight-duration 3000}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :until-straight)
                (phase/tick-subphase robot :follow))]
      (if (phase/phase-done? cmd :until-straight)
        (phase/mark-done cmd)
        cmd))))

(defphase up-to-ramp
  "Drives robot (pointed right) from home side of the table
  up to the ramp, then travelling a significant straight distance across it"
  :init {}
  :chain [[up-to-ramp-find-turn]
          [up-to-ramp-straighten]]
  :tick
  (fn [{:keys [state] :as robot}]
    (phase/tick-chain robot)))

(defphase cross-from-start-by-ramp
  :chain [[up-to-ramp]
          [leave-ramp]]
  :tick (fn [robot] (phase/tick-chain robot)))