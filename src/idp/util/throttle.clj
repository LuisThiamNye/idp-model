(ns idp.util.throttle
  (:require
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
        schedule (fn [g]
                   (.schedule timeout-scheduler ^Runnable g
                     (long timeout) TimeUnit/MILLISECONDS))
        process-queue
        (fn process-queue []
          (when (swap! *promise-queue
                  (fn [q] (if (empty? q) nil q)))
            (let [result (f)
                  _ (schedule process-queue)
                  [q _] (swap-vals! *promise-queue (constantly (enc/queue)))]
              (run! #(deliver q result) q))))]
    (fn []
      (let [p (promise)
            [prev _] (swap-vals! *promise-queue
                       (fn [q]
                         (if q
                           (conj q p)
                           (enc/queue))))]
        (when (nil? prev)
          (let [result (f)]
            (schedule process-queue)
            (deliver p result)))
        @p))))

(comment
  (def t (System/currentTimeMillis))
  
  (def test-fn
    (throttle-noargs-latest
      (fn []
        (let [t' (System/currentTimeMillis)]
          (prn (- t' t))
          (def t t')))
      1))
  
  (future (dotimes [_ 10]
            (test-fn)))
  
  
  )