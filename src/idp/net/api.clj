(ns idp.net.api
  (:require
    [clojure.data :as data]
    [idp.net :as net]
    [idp.robot.client :as client])
  (:import
    (java.net Socket)
    (java.io IOException)))

(defn frac->byte [n]
  (unchecked-int (Math/round (* 255. n))))

(defn as-bitn
  "Converts a boolean to a left-shifted bit"
  [bool lshift]
  (if bool
    (bit-shift-left 1 lshift)
    0))

(defn unsign [n nbytes]
  (if (< n 0)
    (+ n (unchecked-long (Math/pow 256 nbytes)))
    n))


(defn make-insn-byte
  [{:keys [set-motor? get-line-data? get-ultrasonic-data?
           motor1-rev? motor2-rev?]}]
  (bit-or
    (as-bitn set-motor? 7)
    (as-bitn set-motor? 6)
    (as-bitn get-line-data? 5)
    (as-bitn get-ultrasonic-data? 4)
    (as-bitn motor1-rev? 2)
    (as-bitn motor2-rev? 1)))

(defn unsign-byte [n]
  (unsign n 1))

(defn decode-response [^"[B" res-bytes]
  (let [linef-byte (aget res-bytes 0)
        us-byte1 (aget res-bytes 1)
        us-byte2 (aget res-bytes 2)
        us-byte3 (aget res-bytes 3)
        us-byte4 (aget res-bytes 4)]
    {:line-sensor-1 (bit-test linef-byte 7) ; true if white
     :line-sensor-2 (bit-test linef-byte 6)
     :line-sensor-3 (bit-test linef-byte 5)
     :line-sensor-4 (bit-test linef-byte 4)
     :ultrasonic-1
     (+ (unsign (bit-shift-left us-byte1 8) 2)
       (unsign-byte us-byte2))
     :ultrasonic-2
    (+ (unsign (bit-shift-left us-byte3 8) 2)
      (unsign-byte us-byte4))}))

(defn clamp-motor-speed [s]
  (min 255 (max 0 (abs s))))

(defn send-input! [conn input]
  (let [{:keys [motor-1 motor-2 ultrasonic-active?]} input]
    (net/send-bytes! conn
      [(make-insn-byte
         {:set-motor? true
          :motor1-rev? (neg? motor-1)
          :motor2-rev? (neg? motor-2)
          :get-ultrasonic-data? ultrasonic-active?})
       (clamp-motor-speed motor-1)
       (clamp-motor-speed motor-2)
       0 0 0])))

(defn get-response! [conn]
  (some-> (net/read-all-bytes! conn)
    decode-response))

(defrecord NetClient [conn])

(def *client (agent (->NetClient @net/*conn)))

(extend-type NetClient client/Client
  (-get-status [self] (:status self))
  (-get-response! [self]
    (get-response! (:conn self)))
  (-send-input! [self input]
    (send-input! (:conn self) input))
  (-reset-client! [self]
    (net/reset-conn!)
    (send *client
      (fn [client]
        (if (identical? client self)
          (->NetClient @net/*conn)
          client)))))

(comment
  (net/send-bytes!
    [(make-insn-byte {:set-motor? true
                      :motor1-rev? false
                      :motor2-rev? false})
     0 0
     0 0 0])
  
  )