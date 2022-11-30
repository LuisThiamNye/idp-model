(ns idp.robot.brain.detect
  (:require
    [taoensso.encore :as enc]
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
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
  bu/junction-turn-spin
  bu/straight-up-to-blackout
  bu/position-grabber
  bu/tracking-motor-forward
  bu/set-input
  follow/biased-follow
  follow/basic-follow
  follow/follow-up-to-blackout)

(defphase signal-block-density
  :init (fn [params]
          {:density (enc/have #{:high :low} (:density params))
           :time-elapsed 0})
  :tick
  (fn [{:keys [state readings]}]
    (let [signal-duration 4900
          time-elapsed (+ (:time-elapsed state) (:dt readings))]
      (if (<= signal-duration time-elapsed)
        (phase/mark-done
          {:input {:signal-block-density nil}})
        {:state {:time-elapsed time-elapsed}
         :input {:signal-block-density (:density state)}}))))

(defphase measure-density
  :tick
  (fn [{:keys [readings]}]
    (let [density (:block-density readings)]
      (phase/mark-done
        {:output {:block-detected {:density density}}}))))

(defphase detect-block
  "Closes grabber, determines density, and signals the density.
  The grabber remains closed."
  :init {:density nil}
  :chain [[set-input {:signal-block-density nil}]
          [timed-straight {:duration 500}]
          [set-input (motor-input 0)]
          [position-grabber {:grabber-position :closed}]
          [measure-density]
          [signal-block-density]]
  :tick (fn [robot]
          (phase/tick-chain robot
            {:post-tick-fn
             (fn [_ {:keys [output] :as cmd}]
               (if-some [density (:density (:block-detected output))]
                 (phase/merge-cmds cmd
                   {:state {:density density}})
                 cmd))})))

(defphase show-density-live
  :tick
  (fn [{:keys [readings]}]
    {:input {:signal-block-density (:block-density readings)}}))