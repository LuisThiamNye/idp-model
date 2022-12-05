(ns idp.loopthread
  "Utility for creating and controlling a stoppable loop that
  repeatedly executes a function.")

(defn make-loop
  "Creates the state for an indefinitely repeating action that
  can be stopped and restarted. The loop waits for the action to complete.
  `tick-fn` is invoked with each loop cycle, with one argument: the
  time (milliseconds) since the last call to `tick-fn`"
  [tick-fn]
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
                           (catch InterruptedException _
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
