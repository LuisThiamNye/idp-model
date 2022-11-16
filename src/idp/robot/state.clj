(ns idp.robot.state
  (:require
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
    [idp.board.geo :as board.geo]
    [idp.net.api :as net.api]
    [io.github.humbleui.debug :as debug]))

(def *real (atom nil))
(def *sim-speed (atom 1))
(def *sim-dt (atom 5)) ;; in μs

(defn reset-real! []
  (reset! *real
    {:velocity 0 ;; mm/s
     :angular-velocity 0 ;; deg/s
     :position (update (board.params/get-start-box-centre)
                 :y +
                 (let [{:keys [centre-y length]} robot.params/dims]
                   (- centre-y (/ length 2))))
     :angle 270
     
     :line-sensor-1 false
     :line-sensor-2 false
     :line-sensor-3 false
     :line-sensor-4 false
     :ultrasonic-1 {:pos {:x -1 :y -1}
                    :collision-pos {:x -1 :y -1}}
     :ultrasonic-2 {:pos {:x -1 :y -1}
                    :collision-pos {:x -1 :y -1}}
     }))
(reset-real!)

(def initial-input
  {:motor-1 0
   :motor-2 0
   :ultrasonic-active? false})

(def initial-state
  {:readings-history []})

(def initial-readings
  {:line-sensor-1 false
   :line-sensor-2 false
   :line-sensor-3 false
   :line-sensor-4 false
   :line-switches [0 0 0 0]
   :ultrasonic-1 0
   :ultrasonic-2 0})

#_(defrecord Readings [line-sensor-1
                     line-sensor-2
                     line-sensor-3
                     line-sensor-4
                     ultrasonic-1
                     ultrasonic-2])

(defn make-initial-robot []
  {:*readings (atom initial-readings)
   :*state (atom initial-state)
   :*input (atom initial-input)})

(def sim-robot (make-initial-robot))
(def net-robot (make-initial-robot))

(comment
  @(:*readings robot1)
  @(:*input robot1)
  (count (:readings-history @(:*state sim-robot)))
  
  (swap! (:*input robot1) assoc
    :motor-1 0
    :motor-2 0)
  (swap! (:*input robot1) dissoc
    :mode)
  
  (swap! (:*state robot1) assoc :mode :stop)
  
  (reset! (:*readings net-robot) initial-readings)
  (reset! (:*state net-robot) initial-state)
  (reset! (:*input net-robot) initial-input)
  )

(defn get-line-sensors [readings]
  (mapv #(if % :white :black)
    [(:line-sensor-1 readings)
     (:line-sensor-2 readings)
     (:line-sensor-3 readings)
     (:line-sensor-4 readings)]))