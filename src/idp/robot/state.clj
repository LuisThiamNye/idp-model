(ns idp.robot.state
  "Common state for the robots and simulation"
  (:require
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
    [idp.board.geo :as board.geo]
    [idp.net.api :as net.api]
    [io.github.humbleui.debug :as debug]))

(def *real
  "State of the simulated physical world"
  (atom nil))

(def *sim-speed (atom 1))

(def *sim-dt
  "Desired time between each cycle of the simulation
  measured in μs"
  (atom 10000))

(def *sim-block-present? (atom false))
(def *sim-block-density (atom :low))

(defn reset-real! []
  (reset! *real
    {:velocity 0 ;; mm/s
     :angular-velocity 0 ;; deg/s
     :position (update (board.params/get-start-box-centre)
                 :y +
                 (let [{:keys [centre-y length]} robot.params/dims]
                   (- centre-y (/ length 2))))
     :angle 270 ;; measured clockwise from x+ direction
     
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
   :grabber-position :open
   :signal-block-density nil
   :ultrasonic-active? false})

(def initial-state
  {:readings-history []
   :competition-start-time -1})

(def initial-readings
  {:line-sensor-1 false
   :line-sensor-2 false
   :line-sensor-3 false
   :line-sensor-4 false
   :line-switches [0 0 0 0]
   :grabber-moving? false
   :ultrasonic-1 0
   :ultrasonic-2 0})

(defn make-initial-robot []
  {;; Data received from the robot
   :*readings (atom initial-readings)
   ;; Arbitrary state for the client, mainly for autopilot
   :*state (atom initial-state)
   ;; Commands to be sent to the robot
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
  (reset-robot! sim-robot)
  
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