(ns idp.telnet.api
  (:require
    [clojure.data :as data]
    [idp.telnet :as telnet])
  (:import
    (java.net Socket)
    (java.io IOException)))

(defn frac->byte [n]
  (unchecked-int (Math/round (* 255. n))))

(defn as-bitn [bool lshift]
  (if bool
    (bit-shift-left 1 lshift)
    0))

(defn make-insn-byte [{:keys [set-motor? get-line-data? get-imu-data?
                              motor1-rev? motor2-rev?]}]
  (bit-or
    (as-bitn set-motor? 7)
    (as-bitn set-motor? 6)
    (as-bitn get-line-data? 5)
    (as-bitn get-imu-data? 4)
    (as-bitn motor1-rev? 2)
    (as-bitn motor2-rev? 1)))

(defn decode-response [^"[B" res-bytes]
  (let [linef-byte (aget res-bytes 0)]
    {:line-sensor-1 (bit-test linef-byte 7) ; true if white
     :line-sensor-2 (bit-test linef-byte 6)
     :line-sensor-3 (bit-test linef-byte 5)
     :line-sensor-4 (bit-test linef-byte 4)}))


(def *state
  (atom {:line-sensor-1 false
         :line-sensor-2 false
         :line-sensor-3 false}))

(def *input
  (atom {:motor-1 0
         :motor-2 0}))

(def *req-status
  (atom {:waiting? false
         :req-time nil}))

(def response-timeout 500)

(defn reset-connection! []
  (swap! *req-status assoc
    :waiting? false :req-time nil)
  (telnet/reset-conn!))

(defn sendrecv! [conn]
  (let [{:keys [waiting?]} @*req-status]
    (when-not waiting?
      (swap! *req-status assoc
        :waiting? true :req-time (System/currentTimeMillis))
      (let [{:keys [motor-1 motor-2]} @*input]
        (telnet/send-bytes! conn
          [(make-insn-byte {:set-motor? true
                            :motor1-rev? (neg? motor-1)
                            :motor2-rev? (neg? motor-2)})
           (abs motor-1) (abs motor-2)
           0 0 0])))
    (if-some [res (telnet/read-all-bytes! conn)]
      (do
        (reset! *state
          (decode-response res))
        (swap! *req-status assoc
          :waiting? false))
      (when (< response-timeout
              (- (System/currentTimeMillis) (:req-time @*req-status)))
        (println "Response timed out!")
        (reset-connection!)))))

(defn tick! [_dt]
  (assert (.isVirtual (Thread/currentThread)))
  (let [conn @telnet/*conn
        conn-status (:status conn)]
    (if (= :connected conn-status)
      (try
        (let [t1 (System/currentTimeMillis)]
          (sendrecv! conn)
          #_(println "Network time " (- (System/currentTimeMillis) t1)))
        (catch IOException e
          (prn e)
          (reset-connection!)))
      (when-not (= :connecting conn-status)
        (reset-connection!)))))

(comment
  
  (reset! *input
    {:motor-1 100
     :motor-2 100})

  (telnet/send-bytes!
    [(make-insn-byte {:set-motor? true
                      :motor1-rev? false
                      :motor2-rev? false})
     0 0
     0 0 0])
  
  )