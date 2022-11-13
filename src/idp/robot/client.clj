(ns idp.robot.client
  (:import
    (java.io IOException)))

#_(def *req-status
  (atom {:waiting? false
         :req-time nil}))

(def response-timeout 500)

(defprotocol Client
  (-reset-client! [_])
  (-get-status [_])
  (-send-input! [_ input])
  (-get-response! [_]
    "Response of robot including measurements. Nil if not yet available."))

(def ^java.util.Map req-status-atoms
  "client → atom"
  (java.util.WeakHashMap.))

(defn get-req-status-atom [client]
  (locking req-status-atoms
    (or (.get req-status-atoms client)
      (let [res (atom {:waiting? false
                       :req-time nil
                       :id 0})]
        (.put req-status-atoms client res)
        res))))

(defn reset-connection! [client]
  (swap! (get-req-status-atom client) assoc
    :waiting? false :req-time nil)
  (-reset-client! client))

(defn sendrecv!
  [client input]
  (let [*req-status (get-req-status-atom client)
        {:keys [waiting? id]} @*req-status]
    (when-not waiting?
      ; (prn "sending")
      (swap! *req-status assoc
        :waiting? true :req-time (System/currentTimeMillis))
      (-send-input! client
        (assoc input :id id)))
    (if-some [res (-get-response! client)]
      (do
        (swap! *req-status assoc
          :waiting? false
          :id (if (= 255 id) 0 (inc id)))
        res)
      (when (< response-timeout
              (- (System/currentTimeMillis) (:req-time @*req-status)))
        (println "Response timed out!")
        (reset-connection! client)
        nil))))

(defn sync! [client {:keys [*input *readings]}]
  (assert (.isVirtual (Thread/currentThread)))
  (let [client-status (-get-status client)]
    (if (= :connected client-status)
      (try
        (let [t1 (System/currentTimeMillis)]
          (when-some [readings (sendrecv! client @*input)]
            (reset! *readings readings))
          #_(println "Network time " (- (System/currentTimeMillis) t1)))
        (catch IOException e
          (prn e)
          (reset-connection! client)))
      (when-not (= :connecting client-status)
        (reset-connection! client)))))