(ns idp.util.throttle
  (:require
    [idp.util.result :as util.result]
    [taoensso.encore :as enc])
  (:import
    (java.util.concurrent ScheduledExecutorService Executors TimeUnit)))

(def ^ScheduledExecutorService timeout-scheduler
  (Executors/newSingleThreadScheduledExecutor))
(comment
  (.shutdown timeout-scheduler))

(defn throttle-noargs-latest
  "Returns a arity-0 function that acts as f, but may block.
  During timeout period, calls are batched and the next
  call to f is given to all calls.
  `timeout` (ms) specifies delay between end of call of f and
  start of next call"
  [f timeout]
  (let [*promise-queue (atom nil)
        schedule
        (fn [g]
          (let [*fut (promise)]
            (deliver *fut
              (.scheduleWithFixedDelay timeout-scheduler
                ^Runnable (partial g *fut)
                (long timeout) (long timeout) TimeUnit/MILLISECONDS))))
        process-queue
        (fn process-queue [*fut]
          (if (swap! *promise-queue
                  (fn [q] (if (empty? q) nil q)))
            (let [[v success?] (try [(f) true]
                                 (catch Throwable e
                                   [e false]))
                  [q _] (swap-vals! *promise-queue (constantly (enc/queue)))]
              (run! (if success?
                      #(util.result/deliver % v)
                      #(util.result/deliver-ex % v))
                q))
            (.cancel @*fut)))]
    (fn []
      (let [p (util.result/promise)
            [prev _] (swap-vals! *promise-queue
                       (fn [q]
                         (if q
                           (conj q p)
                           (enc/queue))))]
        (when (nil? prev)
          (println "scheduling")
          (util.result/delivering-to p (f))
          (schedule process-queue))
        @p))))

(comment
  (def t (System/currentTimeMillis))
  
  (def test-fn
    (throttle-noargs-latest
      (fn []
        (let [t' (System/currentTimeMillis)]
          (prn (- t' t))
          (def t t')))
      500))
  
  (future (dotimes [_ 10]
            (test-fn)))
  
  )
