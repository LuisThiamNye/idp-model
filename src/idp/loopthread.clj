(ns idp.loopthread)

(defn make-loop [tick-fn]
  (agent
    {:thread nil
     :tick-fn tick-fn}
    :error-mode :continue))

(defn start-loop! [*loop]
  (send *loop
    (fn [{:keys [^Thread thread] :as state}]
      (when thread
        (.interrupt thread))
      (let
        [new-thread
         (.start (Thread/ofVirtual)
           (fn []
             (loop [t (System/currentTimeMillis)]
               (let [state @*loop]
                 (when (= (Thread/currentThread) (:thread state))
                   (let [t2 (System/currentTimeMillis)
                         tick-fn (:tick-fn state)
                         continue?
                         (try
                           (tick-fn (- t2 t))
                           true
                           (catch InterruptedException e
                             false))]
                     (when continue?
                       (recur t2))))))))]
        (assoc state
          :thread new-thread)))))

(defn stop-loop! [*loop]
  (send *loop assoc :thread nil))

(defn loop-running? [*loop]
  (boolean
    (when-some [^Thread t (:thread @*loop)]
      (.isAlive t))))
