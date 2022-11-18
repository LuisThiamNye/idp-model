(ns idp.robot.state
  (:require
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
    [idp.board.geo :as board.geo]
    [idp.net.api :as net.api]
    [io.github.humbleui.debug :as debug]))

(def *real (atom nil))
(def *sim-speed (atom 1))
(def *sim-dt (atom 5000)) ;; in μs

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

(defn reset-robot! [{:keys [*readings *state *input]}]
  (reset! *readings initial-readings)
  (reset! *state initial-state)
  (reset! *input initial-input))

(comment
  @(:*readings robot1)
  @(:*input robot1)
  
  (reset-robot! net-robot)
  
  ;; see if a line sensor ever saw black or not
  (distinct
    (mapv :line-sensor-3
      (:readings-history @(:*state net-robot))))
  )

(defn get-line-sensors
  "Returns readings of line sensors in the order they exist on the
  robot (left to right, from robot's perspective).
  Each reading is either :white or :black"
  [readings]
  (mapv #(if % :white :black)
    [(:line-sensor-1 readings)
     (:line-sensor-2 readings)
     (:line-sensor-3 readings)
     (:line-sensor-4 readings)]))

(defn get-white-line-sensors
  "Returns a vector of boolean indicating whether each line sensor
  is on the line. Order is the same as `get-line-sensors`"
  [readings]
  (mapv #(= :white %) (get-line-sensors readings)))