(ns idp.net.api
  "Implementation of protocol for communicating with the
  robot over the network."
  (:require
    [idp.net :as net]
    [idp.common :as common]
    [idp.robot.client :as client])
  (:import
    (java.io IOException)))

(defn frac->byte
  "Converts 0–1 to a 0–255 int"
  [n]
  (unchecked-int (Math/round (* 255. n))))

(defn as-bitn
  "Converts a boolean to a left-shifted bit"
  [bool lshift]
  (if bool
    (bit-shift-left 1 lshift)
    0))

(defn unsign
  "Maps a signed integer range to an unsigned range
  `nbytes` is the size of the integer type"
  [n nbytes]
  (if (< n 0)
    (+ n (unchecked-long (Math/pow 256 nbytes)))
    n))

(defn unsign-byte [n] (unsign n 1))

(defn as-signed-byte
  "Converts 0–255 to a signed Byte"
  [n]
  (byte (cond-> n (< 127 n) (- 256))))

(defn decode-response
  "Converts the raw bytecode response to a friendly map"
  [^"[B" res-bytes]
  (let [linef-byte (aget res-bytes 0)
        us-byte1 (aget res-bytes 1)
        us-byte2 (aget res-bytes 2)
        us-byte3 (aget res-bytes 3)
        us-byte4 (aget res-bytes 4)]
    {:line-sensors (mapv #(if % :white :black)
                     [(bit-test linef-byte 4)
                      (bit-test linef-byte 5)
                      (bit-test linef-byte 6)
                      (bit-test linef-byte 7)])
     :block-present? (bit-test linef-byte 3)
     :block-density (if (bit-test linef-byte 2)
                      :high :low)
     :grabber-moving? (bit-test linef-byte 1)
     :line-switches
     (let [b1 (aget res-bytes 5)
           b2 (aget res-bytes 6)]
       [(unsign-byte (bit-shift-right b1 4))
        (unsign-byte (bit-and b1 2r1111))
        (unsign-byte (bit-shift-right b2 4))
        (unsign-byte (bit-and b2 2r1111))])
     :ultrasonic-1
     (+ (unsign (bit-shift-left us-byte1 8) 2)
       (unsign-byte us-byte2))
     :ultrasonic-2
    (+ (unsign (bit-shift-left us-byte3 8) 2)
      (unsign-byte us-byte4))}))

(defn clamp-motor-speed [s]
  (min 255 (max 0 (abs s))))

(defn make-insn-byte
  [{:keys [set-motor? get-line-data? get-ultrasonic-data?
           motor1-rev? motor2-rev? close-grabber?]}]
  (bit-or
    (as-bitn set-motor? 7)
    (as-bitn set-motor? 6)
    (as-bitn get-line-data? 5)
    (as-bitn get-ultrasonic-data? 4)
    (as-bitn motor1-rev? 2)
    (as-bitn motor2-rev? 1)
    (as-bitn close-grabber? 0)))

(defn make-request-bytes [input]
  (let [{:keys [motor-1 motor-2 ultrasonic-active? id
                signal-block-density grabber-position]} input]
    [(make-insn-byte
       {:set-motor? true
        :get-line-data? true
        :motor1-rev? (neg? motor-1)
        :motor2-rev? (neg? motor-2)
        :get-ultrasonic-data? ultrasonic-active?
        :close-grabber? (= grabber-position :closed)})
     (clamp-motor-speed motor-1)
     (clamp-motor-speed motor-2)
     (bit-or
       (as-bitn (some? signal-block-density) 7)
       (as-bitn (= :high signal-block-density) 6))
     (as-signed-byte id)
     0]))

(defn send-input! [conn input]
  (let [req-bytes (make-request-bytes input)]
    (common/on-agent (:*socket conn)
      net/send-bytes! req-bytes)))

(defn get-response! [conn]
  (some-> (common/on-agent (:*socket conn)
            net/read-all-bytes!)
    decode-response))

(defrecord NetConnection [conn])

(extend-type NetConnection client/Connection
  (-get-status [{:keys [conn]}]
    (:status conn))
  
  (-get-response! [{:keys [conn]}]
    (when (nil? conn)
      (throw (IOException. "no connection")))
    (get-response! conn))
  
  (-send-input! [{:keys [conn]} input]
    (when (nil? conn)
      (throw (IOException. "no connection")))
    (send-input! conn input))
  )

(def reconnection-throttle-delay 10)
(def *last-reconnection-time (atom 0))

(defrecord NetClient [*conn])

(extend-type NetClient client/Client
  (-get-connection [{:keys [*conn]}]
    (->NetConnection @*conn))
  
  (-reset-connection! [self conn]
    (let [t (System/currentTimeMillis)
          [last-reconnection-time _]
          (swap-vals! *last-reconnection-time
            (fn [lrt]
              (if (< reconnection-throttle-delay (- t lrt))
                t
                lrt)))
          time-remaining (- (+ last-reconnection-time
                              reconnection-throttle-delay)
                           t)]
      (if (pos? time-remaining)
        ;; Throttle the rate at which reconnections can be attempted
        (do (Thread/sleep (long time-remaining))
          (recur self conn))
        (->NetConnection
          @(net/reset-connection! (:conn conn))))))
  )

(def *client (atom (->NetClient net/*state)))