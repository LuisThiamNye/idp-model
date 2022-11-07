(ns idp.robot.brain.travel
  (:require
    [clojure.core.match :refer [match]]
    [idp.robot.command :as cmd]
    [idp.robot.state :as robot.state]))

(def *state
  (atom {:deviation :none}))

(defn tick! []
  (let [{:keys [line-sensor-1
                line-sensor-2
                line-sensor-3]} @robot.state/*real]
    (match [line-sensor-1
            line-sensor-2
            line-sensor-3]
      [_ true _]
      (swap! *state assoc :deviation :none)
      [true false false]
      (swap! *state assoc :deviation :right)
      [false false true]
      (swap! *state assoc :deviation :left)
      :else nil)
    (let [{:keys [deviation]} @*state]
      (case deviation
        :left (cmd/turn-right)
        :none (cmd/turn-straight)
        :right (cmd/turn-left)))))


