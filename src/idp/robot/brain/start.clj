(ns idp.robot.brain.start
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [idp.robot.brain.util :as bu]
    [chic.util.ns :refer [inherit-vars]]
    [clojure.core.match :refer [match]]
    [idp.robot.state :as rs]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input)

(defphase exit-start-turn
  "Once at the start junction, turn 90° in preparation for
  line following"
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [{:keys [readings]
        {:keys [line-triggers]} :state}]
    (let [turn-speed 100
          forward-speed 170
          [_ ntriggers-left ntriggers-right ntriggers-far-right
           :as line-triggers] (mapv + line-triggers (get-line-triggers readings))]
      ;; done when right two (or middle two) sensors have encountered the line
      (if (or (and (pos? ntriggers-right) (pos? ntriggers-far-right))
            (and (pos? ntriggers-right) (pos? ntriggers-left)))
        (phase/mark-done {})
        {:state {:line-triggers line-triggers}
         :input (motor-input forward-speed turn-speed)}))))

(defphase exit-start-find-junction
  "Moves the robot out of the start box until the line sensors
  have found the junction.
  At least three line sensors should find a distinct white region twice,
  or rear ultrasonic gives appropriate reading."
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [{:keys [readings]
        {:keys [line-triggers]} :state}]
    (let
      [forward-speed 200
       line-triggers (mapv + line-triggers (get-line-triggers readings))
       ;; one line sensor may remain on the path and not be triggered twice
       found-junction?
       (let [non-ones (filterv #(not= % 1) line-triggers)]
         (and (<= 2 (count non-ones))
           (every? #(<= 2 %) non-ones)))
       done?
       (or (<= 340 (rs/get-rear-ultrasonic readings) 600)
         (and found-junction?
           (every? #(= % :black) (rs/get-line-sensors readings))))]
      (if done?
        (phase/mark-done {:input {:ultrasonic-active? false}})
        (let [speed forward-speed]
          {:state {:line-triggers line-triggers}
           :input (assoc (motor-input speed)
                    :ultrasonic-active? true)})))))

(defphase exit-start
  "Move from the start box onto the main line"
  :init {}
  :chain [[exit-start-find-junction]
          [timed-straight {:duration 200}]
          [exit-start-turn]]
  :tick (fn [robot] (phase/tick-chain robot)))