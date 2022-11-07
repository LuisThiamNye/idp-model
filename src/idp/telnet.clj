(ns idp.telnet
  (:import
    (java.net Socket InetSocketAddress)
    (java.io
      PrintWriter BufferedReader InputStreamReader
      Reader IOException)))

(def hostname "192.168.137.247")

(def *conn
  (agent
    {:socket nil
     :out nil
     :in nil}
    :error-mode :continue))

(defn reset-conn! []
  (send-off *conn
    (fn [{:keys [^Socket socket ^PrintWriter out ^Reader in]
          :as conn}]
      (do
        (try
          (when socket
            (.close socket))
          (when out
            (.close out))
          (when in
            (.close in))
          (catch Exception _
            (println "failed to close something")))
        (try
          
          (let [socket (Socket.)
                _ (.connect socket
                    (InetSocketAddress. hostname 23)
                    3000 ;; timeout
                    )
                out (PrintWriter. (.getOutputStream socket) false)
                in (BufferedReader. (InputStreamReader. (.getInputStream socket)))]
            (.flush out)
            {:socket socket
             :out out
             :in in})
          (catch Exception e
            (println "Failed to initialise socket")
            (prn e)
            conn))))))

(defn send-line! [s]
  (let [{:keys [^PrintWriter out]} @*conn]
    (.println out s)
    (.flush out)))

(defn read-all! []
  (let [{:keys [^Reader in]} @*conn]
    (when in
      (let [sb (StringBuilder.)]
        (loop []
          (when (.ready in)
            (.appendCodePoint sb (.read in))
            (recur)))
        (str sb)))))


(reset-conn!)

(send-line! "m1a")

(read-all!)

@*conn

(char 255)

; set motor speed (n, speed)
; get-data => line-follower (3-4), IMU (6), distance/ultrasound
; LDR
; set LEDs - high/low ρ
; grabber arm

;; length encoding

"

m1_m2_

Message format:

*[
1b: command type - m
...
]

Response format:
6*4b: accelerometer (X Y Z AngX AngY AngZ)
1b: line sensor permutation
"