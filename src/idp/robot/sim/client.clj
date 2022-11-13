(ns idp.robot.sim.client
  (:require
    [taoensso.encore :as enc]
    [idp.robot.client :as client]
    [idp.robot.params :as robot.params]
    [idp.robot.state :as robot.state]))

(defn clamp-motor-speed [s]
  (min 255 (max 0 (abs s))))

(defrecord SimClient [*state])

(def initial-client-state
  {:status :connected
   :req-queue (enc/queue)
   :res-queue (enc/queue)
   :ready-res-queue (enc/queue)
   :ready-req-queue (enc/queue)})

(def initial-brain-state
  {:last-insn-time -1
   :paused? false
   :latest-readings robot.state/initial-readings})

(def *client
  (agent
    (->SimClient
      (atom (merge initial-client-state
              initial-brain-state)))))

(defn rand-send-latency []
  (+ 20 (rand-int 160)))
(defn rand-response-latency []
  (rand-send-latency))
(defn rand-request-drop? []
  (< (rand) 0.5))
(defn rand-response-drop? []
  (rand-request-drop?))

(extend-type SimClient client/Client
  (-get-status [self] (:status @(:*state self)))
  
  (-get-response! [{:keys [*state]}]
    (let [q (:ready-res-queue @*state)]
      (when (< 0 (count q))
        (swap! *state update :ready-res-queue pop)
        (peek q))))
  
  (-send-input! [{:keys [*state]} input]
    (when-not (rand-request-drop?)
      (swap! *state update :req-queue conj
        {:timestamp (System/currentTimeMillis)
         :latency (rand-send-latency)
         :message input})))
  
  (-reset-client! [self]
    (swap! (:*state self) merge
      (assoc initial-client-state
        :status :connected))
    #_(send *client
      (fn [client]
        (if (identical? client self)
          (->SimClient (atom (assoc initial-client-state
                               :status :connected)))
          client)))))

(defn send-response! [*state response]
  (when-not (rand-response-drop?)
    (swap! *state update :res-queue conj
      {:timestamp (System/currentTimeMillis)
       :latency (rand-response-latency)
       :message response}))
  (swap! *state assoc-in [:latest-readings :line-switches] [0 0 0 0]))

(defn process-input! [input]
  (let [{:keys [motor-1 motor-2]} input
        {:keys [max-rpm wheel-diameter wheel-spacing]}
        robot.params/dims
        motor->v
        (fn [spd]
          (let [rpm (* max-rpm (max 0 (min 1 (/ spd 255))))
                rps (/ rpm 60)
                circumference (* Math/PI wheel-diameter)]
            (* circumference rps)))
        motor1-v (motor->v motor-1)
        motor2-v (motor->v motor-2)]
    (swap! robot.state/*real assoc
      :velocity (/ (+ motor1-v motor2-v) 2)
      :angular-velocity
      (* (/ (- motor1-v motor2-v)
           wheel-spacing)
        (/ 180 Math/PI)))))

(defn process-queues [*state fromqk toqk]
  (let [state @*state
        fromq (fromqk state)
        pkg (peek fromq)]
    (when (and pkg
            (<= (+ (:latency pkg) (:timestamp pkg))
              (System/currentTimeMillis)))
      (let [msg (:message pkg)]
        (swap! *state #(-> %
                         (update fromqk pop)
                         (update toqk conj msg)))
        msg))))

(defn update-line-switches [readings readings']
  (update (merge readings readings')
    :line-switches
    (fn [switches]
      (mapv (fn [n prev current]
              (when (not= prev current)
                #_(prn "hop" readings readings'))
              (cond-> n (not= prev current) inc))
        switches
        [(:line-sensor-1 readings)
         (:line-sensor-2 readings)
         (:line-sensor-3 readings)
         (:line-sensor-4 readings)]
        [(:line-sensor-1 readings')
         (:line-sensor-2 readings')
         (:line-sensor-3 readings')
         (:line-sensor-4 readings')]))))

(def rc-timeout 300)

(defn pause-activities! []
  (process-input! {:motor-1 0 :motor-2 0}))

(defn resume-activities! [])

(defn tick! []
  (let [{:keys [*state]} @*client
        state @*state
        paused? (:paused? state)
        readings (:latest-readings state)
        readings'
        (select-keys @robot.state/*real
          [:line-sensor-1
           :line-sensor-2
           :line-sensor-3
           :line-sensor-4
           :ultrasonic-1
           :ultrasonic-2])
        readings' (update-line-switches readings readings')]
    (swap! *state assoc :latest-readings readings')
    (if-some [input (process-queues *state :req-queue :ready-req-queue)]
      (do
        (swap! *state assoc :last-insn-time (System/currentTimeMillis))
        (when paused?
          (swap! *state assoc :paused? false)
          (resume-activities!))
        (swap! *state update :ready-req-queue pop)
        (process-input! input)
        (send-response! *state readings'))
      (when (and (< rc-timeout (- (System/currentTimeMillis)
                                 (:last-insn-time state)))
              (not paused?))
        (pause-activities!)))
    (process-queues *state :res-queue :ready-res-queue)))