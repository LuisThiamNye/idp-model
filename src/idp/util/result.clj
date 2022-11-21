(ns idp.util.result
  (:refer-clojure :exclude [deliver promise]))

(defn promise
  "Like clojure.core/promise, but exceptions computing the result
  may be delivered and thrown upon dereferencing.

  Safe to substitute in place of a clojure.core/promise."
  []
  (let [latch (java.util.concurrent.CountDownLatch. 1)
        *success (volatile! false)
        *value (atom latch)]
    (reify
      clojure.lang.IDeref
      (deref [_]
        (.await latch)
        (let [v @*value]
          (if @*success v (throw v))))
      
      clojure.lang.IBlockingDeref
      (deref [_ timeout-ms timeout-val]
        (if (.await latch timeout-ms
              java.util.concurrent.TimeUnit/MILLISECONDS)
          (let [v @*value]
            (if @*success v (throw v)))
          timeout-val))
      
      clojure.lang.IPending
      (isRealized [_] (zero? (.getCount latch)))
      
      clojure.lang.IFn
      (invoke [self value] (.invoke self value true))
      (invoke [self value success?]
        (when (and (pos? (.getCount latch))
                (compare-and-set! *value latch value))
          (vreset! *success success?)
          (.countDown latch)
          self)))))

(defn deliver [p v] (p v true))
(defn deliver-ex [p ex] (p ex false))

(defmacro delivering-to
  "Delivers the result of the body to the promise,
  including anything thrown."
  [p & body]
  `(let [p# ~p]
     (try
       (deliver p# (do ~@body))
       (catch Throwable e#
         (deliver-ex p# e#)))))
