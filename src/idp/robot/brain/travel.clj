(ns idp.robot.brain.travel
  (:require
    [idp.telnet.api :as api]
    [clojure.core.match :refer [match]]
    [idp.robot.command :as cmd]
    [idp.robot.state :as robot.state]))

(def *state
  (atom {:mode :exit-start
         :over-horiz? false
         :n-horiz-found 0
         :deviation :none}))

#_(defn tick-simple-follow []
  (let [{:keys [line-sensor-1
                line-sensor-2
                line-sensor-3]} @api/*state]
    (match [line-sensor-left
            line-sensor-centre
            line-sensor-right]
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
      (case deviation
        :left
        (swap! api/*input assoc
          :motor-1 (+ sturnf sturn)
          :motor-2 (+ sturnf (- sturn)))
        :none
        (swap! api/*input assoc
          :motor-1 sforward
          :motor-2 sforward)
        :right
        (swap! api/*input assoc
          :motor-1 (+ sturnf (- sturn))
          :motor-2 (+ sturnf sturn))))))

(defn tick-exit-start []
  (let
    [{:keys [line-sensor-1
             line-sensor-2
             line-sensor-3
             line-sensor-4]} @api/*state
     {:keys [over-horiz?
             n-horiz-found]} @*state
     horiz? (<= 3
              (mapv #(if % 1 0)
                [line-sensor-1
                 line-sensor-2
                 line-sensor-3
                 line-sensor-4]))]
    (match [horiz? over-horiz?]
      [true true]
      nil
      [true false] ;; entered horiz
      (if (<= 1 n-horiz-found)
        ;; found the T-junction
        (swap! *state assoc :mode :stop)
        ;; found the box edge
        (swap! *state assoc
          :over-horiz? true
          :n-horiz-found (inc n-horiz-found)))
      [false true] ;; exit horiz
      (swap! *state :over-horiz? false)
      [false false]
      nil)))

(defn tick! []
  (case (:mode @*state)
    :exit-start
    (tick-exit-start)
    :stop
    (swap! api/*input assoc
      :motor-1 0 :motor-2 0)))



