(ns idp.robot.command
  (:require
    [idp.robot.state :as state]))

(defn go-forwards []
  (swap! state/*real assoc :velocity 100))

(def std-angular-v 40)

(defn turn-right []
  (swap! state/*real assoc :angular-velocity std-angular-v))
(defn turn-left []
  (swap! state/*real assoc :angular-velocity (- std-angular-v)))
(defn turn-straight []
  (swap! state/*real assoc :angular-velocity 0))

(defn stop []
  (swap! state/*real assoc :velocity 0 :angular-velocity 0))

(comment
  (go-forwards)
  (turn-right)
  (turn-left)
  (turn-straight)
  (stop)
  (state/reset-real!)
  (swap! state/*real assoc :angle 90)
  
  (state/tick! 1)
  )