(ns idp.telnet
  (:require
    [clojure.java.shell :as shell])
  (:import
    (java.net Socket InetSocketAddress SocketException)
    (java.io
      PrintWriter BufferedReader InputStreamReader
      Reader IOException InputStream OutputStream)))

(def ^String hostname "192.168.137.222")

(defn get-server-ip []
  (let [{:keys [exit out]} (shell/sh "arp" "-a")
        [_ ip] (re-find #"\s(\S+)\s+84-cc-a8-2e-96-18" out)]
    ip))

(defn wait-for-ip []
  (loop []
    (if-some [ip (get-server-ip)]
      ip
      (do (Thread/sleep 50)
        (recur)))))

(def *conn
  (agent
    {:socket nil
     :out nil
     :in nil
     :status :disabled}
    :error-mode :continue))

(defn reset-conn! []
  (send-off *conn
    (fn [{:keys [^Socket socket ^OutputStream out ^InputStream in
                 ^Thread connecting-thread]
          :as conn}]
      (try
        (when socket
          (.close socket)
          (when out (.close out))
          (when in (.close in)))
        (catch Exception e
          (println "failed to close something")
          (prn e)))
      (try
        (when connecting-thread
          (.interrupt connecting-thread)
          (.join connecting-thread))
        (catch Exception e
          (prn e)))
      (let [socket (Socket.)]
        {:socket socket
         :out nil :in nil
         :status :connecting
         :connecting-thread
         (.start (Thread/ofVirtual)
           (fn []
             (let
               [hostname (wait-for-ip)
                new-conn
                (try
                  (println "Connecting to " hostname)
                  (.connect socket
                    (InetSocketAddress. hostname 23))
                  (let [out (.getOutputStream socket)
                        in (.getInputStream socket)]
                    (.flush out)
                    {:socket socket
                     :out out :in in
                     :status :connected})
                  (catch SocketException e
                    (if (Thread/interrupted)
                      (println "Connection attempt interrupted")
                      (do (println "Connection attempt failed")
                        (prn e)))
                    nil)
                  (catch Exception e
                    (prn e)))]
               (send *conn
                 (fn [{:as conn}]
                   (if (identical? (:socket conn) socket)
                     (if new-conn
                       (do
                         (println "Connection successful")
                         new-conn)
                       {:socket socket
                        :out nil :in nil
                        :status :failed})
                     conn))))))}))))

(defn send-bytes! [conn s]
  (let [{:keys [^OutputStream out]} conn]
    (.write out
      (byte-array s))
    (.flush out)))

(defn read-all-bytes! [conn]
  (let [{:keys [^InputStream in]} conn]
    (when in
      (let [nbytes (.available in)]
        (when (< 0 nbytes)
          (.readNBytes in nbytes))))))

(defn read-response! ^"[B" [conn]
  (let [{:keys [^InputStream in]} conn]
    (when in
      (let [res-ba (byte-array 8)
            nbytes (.read in res-ba)]
        (when (< 0 nbytes)
          res-ba)))))


(comment
  (reset-conn!)
  
  84-cc-a8-2e-96-18
  
  )


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