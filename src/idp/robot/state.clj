(ns idp.robot.state
  "Common state for the robots and simulation"
  (:require
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]))

(def *real
  "State of the simulated physical world"
  (atom nil))

(def *sim-speed (atom 1))

(def *sim-dt
  "Desired time between each cycle of the simulation
  measured in μs"
  (atom 10000))

(def *sim-block-present? (atom false))
(def *sim-block-density (atom :low)) ;; :high, :low

(defn reset-real!
  "Restores the simulated world to its initial state"
  []
  (reset! *real
    {:velocity 0 ;; mm/s
     :angular-velocity 0 ;; deg/s clockwise
     ;; Initial position is centred within the start box (measured in millimetres)
     :position (update (board.params/get-start-box-centre)
                 :y +
                 (let [{:keys [centre-y length]} robot.params/dims]
                   (- centre-y (/ length 2))))
     :angle 270 ;; measured clockwise from x+ direction, in degrees
     
     :line-sensors [:black :black :black :black] ;; :black, :white
     :ultrasonic-1 {;; Location of the sensor
                    :pos {:x -1 :y -1}
                    ;; Point of the obstacle that the sensor is pointing at
                    :collision-pos {:x -1 :y -1}}
     :ultrasonic-2 {:pos {:x -1 :y -1}
                    :collision-pos {:x -1 :y -1}}
     }))
(reset-real!)

(def initial-input
  {:motor-1 0 ;; [-255,255]
   :motor-2 0
   :grabber-position :open ;; :open, :closed
   :signal-block-density nil ;; nil (both LEDs off), :high, :low
   ;; Whether the Arduino should take ultrasonic
   ;; distance readings and respond with the results
   :ultrasonic-active? false})

(def initial-state
  {:auto? false ;; Whether autopilot is active
   ;; Vector of all previous readings, with latest readings at the end
   :readings-history []})

(def initial-readings
  {:line-sensors [:black :black :black :black] ;; :black, :white
   ;; Number of times the line reading changed since last response
   :line-switches [0 0 0 0]
   :grabber-moving? false
   :block-density nil ;; :high, :low
   ;; Ultrasonic reading of zero means a failure to get a reading
   ;; Otherwise represents a distance in millimetres
   :ultrasonic-1 0
   :ultrasonic-2 0})

(defn make-initial-robot []
  {;; Data received from the robot
   :*readings (atom initial-readings)
   ;; Arbitrary state for the client, mainly for autopilot
   :*state (atom initial-state)
   ;; Commands to be sent to the robot
   :*input (atom initial-input)})

(def sim-robot "State of the simluated robot"
  (make-initial-robot))
(def net-robot "State of the real, network-connected robot"
  (make-initial-robot))

(defn reset-robot!
  "Restores the initial states of a robot"
  [{:keys [*readings *state *input]}]
  (reset! *readings initial-readings)
  (reset! *state initial-state)
  (reset! *input initial-input))

(defn get-line-sensors
  "Returns readings of line sensors in the order they exist on the
  robot (left to right, from robot's perspective).
  Each reading is either :white or :black"
  [readings]
  (:line-sensors readings))

(defn get-white-line-sensors
  "Returns a vector of booleans indicating whether each line sensor
  is on the line. Order is the same as `get-line-sensors`"
  [readings]
  (mapv #(= :white %) (get-line-sensors readings)))

(defn get-active-dt
  "The duration of time (in ms) since the previous response that
  the robot was operating in an unpaused state.
  May be an estimation"
  [readings]
  (min (:dt readings) robot.params/rc-timeout))

(defn get-left-ultrasonic
  "Returns a distance in millimetres, or zero if there is no reading"
  [readings]
  (:ultrasonic-1 readings))

(defn get-rear-ultrasonic
  "Returns a distance in millimetres, or zero if there is no reading"
  [readings]
  (:ultrasonic-2 readings))