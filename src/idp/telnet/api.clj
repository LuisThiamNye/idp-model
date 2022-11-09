(ns idp.telnet.api
  (:require
    [clojure.data :as data]
    [idp.telnet :as telnet]))

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
     :line-sensor-3 (bit-test linef-byte 5)}))


(def *state
  (atom {:line-sensor-1 false
         :line-sensor-2 false
         :line-sensor-3 false}))

(def *input
  (atom {:motor-1 0
         :motor-2 0}))

(defn tick! [_dt]
  (let [{:keys [motor-1 motor-2]} @*input]
    (telnet/send-bytes!
     [(make-insn-byte {:set-motor? true
                       :motor1-rev? (neg? motor-1)
                       :motor2-rev? (neg? motor-2)})
      (abs motor-1) (abs motor-2)
      0 0 0]))
  (let [res (telnet/read-all-bytes!)]
    (when res
      (reset! *state
        (decode-response res)))))



(comment
  (set-motor-speed 1 255)
  
  (tick! 0)
  (reset! *input
    {:motor-1 100
     :motor-2 100})

  (telnet/send-bytes!
    [(make-insn-byte {:set-motor? true
                      :motor1-rev? false
                      :motor2-rev? false})
     0 0
     0 0 0])
  
  (let [_ (telnet/send-bytes!
            [(make-insn-byte {:set-motor? true
                              :motor1-rev? false
                              :motor2-rev? false})
             0 0
             0 0 0])
        _ (Thread/sleep 200)
        res (telnet/read-all-bytes!)]
    (when res
      (decode-response res)))
  
  )