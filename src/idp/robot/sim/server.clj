(ns idp.robot.sim.server
  (:require
    [idp.loopthread :as loopth])
  (:import
    (java.io IOException)
    (java.net ServerSocket Socket)))

(def ^ServerSocket server-socket
  (ServerSocket. 7890))

(def *client (atom nil))

(defn respond-to-client [^Socket client]
  (let [in (.getInputStream client)
        out (.getOutputStream client)
        nbytes 6]
    (vec (.readNBytes in nbytes))
    ; (Thread/sleep (int (rand-int 100)))
    (when (< (rand) 1)
      (.write out (byte-array 7))
      (.flush out))))

(defn get-connected-client ^Socket []
  (let [c ^Socket @*client]
    (if (and c (.isConnected c) (not (.isClosed c)))
      c
      (do
        (println "Server: waiting for client")
        (when c (.close c))
        (reset! *client
          (.accept server-socket))))))

(defn tick! [_dt]
  (Thread/sleep 10)
  (let [client (get-connected-client)]
    (try (respond-to-client client)
      (catch IOException e
        (.close @*client)
        (println "Server: error when responding:" (.getMessage e))))))

(def responder
  (loopth/make-loop #'tick!))

(comment
  (loopth/start-loop! responder)
  (loopth/stop-loop! responder)
  
  (do
    (.close @*client)
    (.close server-socket))
  
  
  )