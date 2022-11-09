(ns idp.robot.brain.travel
  (:require
    [idp.telnet.api :as api]
    [clojure.core.match :refer [match]]
    [idp.robot.command :as cmd]
    [idp.robot.state :as robot.state]))

(def *state
  (atom {:deviation :none}))

(defn tick! []
  (let [{:keys [line-sensor-1
                line-sensor-2
                line-sensor-3]} #_@robot.state/*real
        @api/*state]
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
    (let [{:keys [deviation]} @*state
          sturn 70
          sturnf 30
          sforward 150]
      #_(case deviation
        :left (cmd/turn-right)
        :none (cmd/turn-straight)
        :right (cmd/turn-left))
      (case deviation
        :left
        (do (prn "turn right")
          (swap! api/*input assoc
            :motor-1 (+ sturnf sturn)
            :motor-2 (+ sturnf (- sturn))))
        :none
        (do (prn "turn forwards")
          (swap! api/*input assoc
            :motor-1 sforward
            :motor-2 sforward))
        :right
        (do (prn "turn left")
          (swap! api/*input assoc
            :motor-1 (+ sturnf (- sturn))
            :motor-2 (+ sturnf sturn)))))))


