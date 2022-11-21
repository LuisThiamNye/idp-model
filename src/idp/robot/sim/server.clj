(ns idp.robot.sim.server
  "Server logic of the simulated robot.
  Mimics code that would run on the Arduino"
  (:require
    [idp.robot.sim.device :as device]
    [idp.robot.state :as robot.state]))

(defprotocol ServerConnection
  "Used by the simulated server to communicate with the client"
  (-get-request [_])
  (-send-response [_ response]))

(defn combine-prev-response
  "Accumulates line sensor data from previous readings
  into the current response."
  [{:keys [sent-readings latest-readings] :as state}]
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
              switches)))))))

(defn propagate-response [state]
  (-> state
    (assoc :sent-readings (:latest-readings state))
    (assoc-in [:latest-readings :line-switches] [0 0 0 0])))

(defn send-response! [*state]
  (when (:retry? @*state)
    (swap! *state combine-prev-response))
  (let [response (:latest-readings @*state)]
    (-send-response (:conn @*state) response)
    (swap! *state propagate-response)))

(defn update-line-switches
  "Adds :line-switches data based on previous readings.
  (Number of times line sensors changed since previous response)"
  [prev-readings readings]
  (assoc readings
    :line-switches
    (let [switches (:line-switches prev-readings)]
      (mapv (fn [n prev current]
              (cond-> n (not= prev current) inc))
        switches
        [(:line-sensor-1 prev-readings)
         (:line-sensor-2 prev-readings)
         (:line-sensor-3 prev-readings)
         (:line-sensor-4 prev-readings)]
        [(:line-sensor-1 readings)
         (:line-sensor-2 readings)
         (:line-sensor-3 readings)
         (:line-sensor-4 readings)]))))

(defn process-request! [*state req]
  ;; important to use id rather than :retry? bit or else cannot
  ;; distinguish between dropped messages on req/response
  (swap! *state
    (fn [state]
      (assoc state
        :retry? (= (:req-id state) (:id req))
        :req-id (:id req))))
  (device/process-input! req))

(def rc-timeout 100)

(defn pause-activities! []
  (device/process-input! {:motor-1 0 :motor-2 0}))

(defn resume-activities!
  "Does nothing as motors get set on every request anyway"
  [])

(def initial-brain-state
  {:conn nil
   :last-insn-time -1
   :paused? false
   ; :req-id 255
   :latest-readings robot.state/initial-readings})

(def *state (atom nil))
(swap! *state merge initial-brain-state)

(defn tick! []
  (let [state @*state
        paused? (:paused? state)]
    (swap! *state assoc :latest-readings
      (update-line-switches (:latest-readings state) (device/take-readings)))
    (if-some [input (-get-request (:conn state))]
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
        (pause-activities!)))))