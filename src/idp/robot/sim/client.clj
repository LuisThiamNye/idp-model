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
   ; :req-id 255
   :latest-readings robot.state/initial-readings})

(def *client
  (agent
    (->SimClient
      (atom (merge initial-client-state
              initial-brain-state)))))

(reset! (:*state @*client)
  (merge initial-client-state
    initial-brain-state))

(defn rand-send-latency []
  (+ 20 (rand-int 60)))
(defn rand-response-latency []
  (rand-send-latency))
(defn rand-request-drop? []
  (< (rand) 0.05))
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

(defn send-response! [*state]
  (when (:retry? @*state)
    (swap! *state
      (fn [{:keys [sent-readings latest-readings] :as state}]
        (if (nil? sent-readings)
          state
          (-> state
            (dissoc :sent-readings)
            (assoc :latest-readings
              (update latest-readings :line-switches
               (fn [switches]
                 (mapv (fn [prev cur]
                         (+ prev cur))
                   (:line-switches sent-readings)
                   switches)))))))))
  (let [response (:latest-readings @*state)]
    (when-not (rand-response-drop?)
      (swap! *state update :res-queue conj
        {:timestamp (System/currentTimeMillis)
         :latency (rand-response-latency)
         :message response}))
    (swap! *state
      (fn [state]
        (-> state
          (assoc :sent-readings response)
          (assoc-in [:latest-readings :line-switches] [0 0 0 0]))))))

(defn process-input! [input]
  (let
    [{:keys [motor-1 motor-2]} input
     {:keys [max-rpm wheel-diameter wheel-spacing]}
     robot.params/dims
     motor-thres 40
     motor->v
     (fn [spd]
       (let
         [rpm (* max-rpm (max 0 (min 1 (/ (if (neg? spd)
                                            (min 0 (+ spd motor-thres))
                                            (max 0 (- spd motor-thres)))
                                           255))))
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

(defn process-request! [*state req]
  ;; important to use id rather than :retry? bit or else cannot
  ;; distinguish between dropped messages on req/response
  (swap! *state
    (fn [state]
      (assoc state
        :retry? (= (:req-id state) (:id req))
        :req-id (:id req))))
  (process-input! req))

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
        (process-request! *state input)
        (send-response! *state))
      (when (and (< rc-timeout (- (System/currentTimeMillis)
                                 (:last-insn-time state -1)))
              (not paused?))
        (pause-activities!)))
    (process-queues *state :res-queue :ready-res-queue)))