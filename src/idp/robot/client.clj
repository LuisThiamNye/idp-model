(ns idp.robot.client
  (:import
    (java.io IOException)))

#_(def *req-status
  (atom {:waiting? false
         :req-time nil}))

(def response-timeout 200)
; (def response-timeout ##Inf)

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
      (let [res (atom {:req-time nil
                       :status :connecting
                       :requests-dropped 0
                       :requests-completed 0
                       :id 0})]
        (.put req-status-atoms client res)
        res))))

(defn reset-connection! [client]
  (swap! (get-req-status-atom client) assoc
    :status :connecting
    :req-time nil)
  (-reset-client! client))

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

(defn sendrecv!
  [client input]
  (let
    [*req-status (get-req-status-atom client)
     {:keys [status id]} @*req-status
     want-to-send? (not= :waiting status)
     sent?
     (when want-to-send?
       (try
         (-send-input! client
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
     failed-to-send? (and want-to-send? (not sent?))
     response (when-not failed-to-send?
                (try (-get-response! client)
                  (catch IOException _)))]
    (when-not response
      (cond
        failed-to-send?
        (do (println "net.api: Failed to send")
          (reset-connection! client))
        (let [{:keys [req-time]} @*req-status]
          (when req-time
            (< response-timeout
              (- (System/currentTimeMillis) req-time))))
        (do
          (println "net.api: Response timed out!")
          (swap! *req-status update :requests-dropped inc)
          (reset-connection! client)
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
        (println "net.api: Not connected; connecting")
        (reset-connection! client)))))