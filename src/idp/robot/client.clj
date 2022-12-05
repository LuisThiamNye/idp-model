(ns idp.robot.client
  "The main client behaviour for exchanging data with an abstract robot.
  Contains logic for sending requests and listening for responses."
  (:import
    (java.io IOException)))

(def response-timeout
  "After this many milliseconds waiting for a reponse,
  attempt to reset the connection."
  160)

;; One client per socket connection
(defprotocol Connection
  (-get-status [_]
    "Could be: :disabled, :connecting, :connected, :failed")
  (-send-input! [_ input])
  (-get-response! [_]
    "Response of robot including measurements. Nil if not yet available."))

(defprotocol Client
  (-get-connection [_]
    "Returned object implements Connection protocol")
  (-reset-connection! [_ conn]
    "Blocks while ending the provided Connection.
    Returns the new connection or nil if provided connection is has
    already been replaced by a new one."))

(def ^java.util.Map req-status-atoms
  "Additional state about a client.
  Map of client → atom"
  (java.util.WeakHashMap.))

(defn get-req-status-atom [client]
  (locking req-status-atoms
    (or (.get req-status-atoms client)
      (let [res (atom {:req-time nil
                       :status :none
                       :requests-dropped 0
                       :requests-completed 0
                       :id 0})]
        (.put req-status-atoms client res)
        res))))

(defn reset-connection! [client conn]
  (swap! (get-req-status-atom client) assoc
    :status :connecting
    :req-time nil)
  (-reset-connection! client conn))

(defn conform-input
  [{:keys [motor-1 motor-2] :as input}]
  (-> input
    (cond-> (< 255 motor-1)
      (assoc :motor-1 255))
    (cond-> (< 255 motor-2)
      (assoc :motor-2 255))
    (cond-> (< motor-1 -255)
      (assoc :motor-1 -255))
    (cond-> (< motor-2 -255)
      (assoc :motor-2 -255))))

(defn attempt-send!
  "Attempts to send input to the Arduino, updating
  status as necessary.
  Returns true if successful."
  [conn *req-status id input]
  (try
    (-send-input! conn
      (assoc (conform-input input) :id id))
    (swap! *req-status assoc
      :status :waiting
      :req-time (System/currentTimeMillis))
    true
    (catch IOException _
      (swap! *req-status assoc
        :status :failed
        :req-time nil)
      false)))

(defn sendrecv!
  "Handles sending or receiving of data as appropriate.
  Does not block.
  If a response was obtained, this is returned. Else nil."
  [client conn input]
  (let
    [*req-status (get-req-status-atom client)
     {:keys [status id]} @*req-status
     want-to-send? (not= :waiting status)
     sent?
     (when want-to-send?
       (attempt-send! conn *req-status id input))
     failed-to-send? (and want-to-send? (not sent?))
     response (when-not failed-to-send?
                (try (-get-response! conn)
                  (catch IOException _)))]
    (when-not response
      (cond
        failed-to-send?
        (do (println "client: Failed to send")
          (reset-connection! client conn))
        
        ;; If response timed out
        (let [{:keys [req-time]} @*req-status]
          (when req-time
            (< response-timeout
              (- (System/currentTimeMillis) req-time))))
        (do
          (swap! *req-status update :requests-dropped inc)
          (reset-connection! client conn)
          nil)))
    (when response
      (swap! *req-status
        (fn [s]
          (-> s
            (update :requests-completed inc)
            (assoc
              :status :connected
              :id (if (= 255 id) 0 (inc id))))))
      response)))

(defn reset-readings [prev-readings readings input]
  {:post [(= 4 (count (:line-switches %)))]}
  (-> readings
    ;; ensure that :line-switches is consistent with previous readings
    ;; by incrementing them if necessary
    (assoc :line-switches
      (mapv (fn [prev-ls nswitches ls]
              (let [expected-change? (odd? nswitches)
                    change? (not= prev-ls ls)]
                (cond-> nswitches
                  (not= change? expected-change?)
                  inc)))
        (:line-sensors prev-readings [:black :black :black :black])
        (:line-switches readings)
        (:line-sensors readings)))
    ;; Set ultrasonic readings to zero (invalid) if Arduino was not
    ;; requested to take those readings. This prevents misleading the
    ;; navigation logic
    (cond-> (not (:ultrasonic-active? input))
      (assoc :ultrasonic-1 0 :ultrasonic-2 0))))

(defn sync!
  "Main function that should get executed each clock cycle
  to handle the transfer of data between the client and
  the Arduino"
  [client {:keys [*input *readings]}]
  (let [conn (-get-connection client)
        req-status @(get-req-status-atom client)
        conn-status (-get-status conn)]
    (if (= :connected conn-status)
      (try
        (let [input @*input]
          (when-some [readings (sendrecv! client conn input)]
            (swap! *readings reset-readings readings input)))
        (catch IOException e
          (prn e)
          (reset-connection! client conn)))
      (when-not (= :connecting conn-status)
        (when-not (= :connecting (:status req-status))
          (println "client: Not connected; connecting"))
        (reset-connection! client conn)))))