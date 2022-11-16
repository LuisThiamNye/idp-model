(ns idp.net "
Contains primitives for connecting to and transmitting data
to the Arduino."
  (:require
    [clojure.java.shell :as shell])
  (:import
    (java.net Socket InetSocketAddress SocketException
      SocketTimeoutException)
    (java.io
      PrintWriter BufferedReader InputStreamReader
      Reader IOException InputStream OutputStream)))

(def server-mac-address "84-cc-a8-2e-96-18")

(defn get-server-ip
  "Finds the IP address of the Arduino connected to the current
Windows device via mobile hotspot."
  []
  (let [{:keys [out]} (shell/sh "arp" "-a")
        [_ ip] (re-find (re-pattern (str #"\s(\S+)\s+" server-mac-address)) out)]
    ip))

(defn wait-for-ip! ^String [continue-fn]
  (loop []
    (or (get-server-ip)
      (when (continue-fn)
        (Thread/sleep 5)
        (recur)))))

(defn conn-err-handler [*conn e]
  (println "Agent error")
  (prn e))

(def *conn
  (agent
    {:socket nil
     :out nil
     :in nil
     :status :disabled}
    :error-mode :continue
    :error-handler conn-err-handler))

(def socket-connect-timeout 2000)

(defn close-conn [conn]
  (let [{:keys [^Socket socket ^OutputStream out ^InputStream in
                ^Thread connecting-thread]} conn]
    (try
      (when socket
        (.close socket))
      (catch Exception e
        (println "Net: failed to close something")
        (prn e)))
    (when connecting-thread
      (.join connecting-thread 1000)
      (when (.isAlive connecting-thread)
        (println "Net: connecting thread timed out, interrupting...")
        (.interrupt connecting-thread)))
    (assoc conn
      :status :disabled
      :socket nil
      :out nil
      :in nil
      :connecting-thread nil)))

(defn connect-conn []
  (let [socket (Socket.)]
    {:socket socket
     :out nil :in nil
     :status :connecting
     :connecting-thread
     (.start (Thread/ofVirtual)
       (fn []
         (let
           [hostname (wait-for-ip!
                       #(= (Thread/currentThread)
                          (:connecting-thread @*conn)))
            new-conn
            (try
              (println "Net: Connecting to " hostname)
              (.connect socket
                (InetSocketAddress. hostname 23)
                socket-connect-timeout)
              (let [out (.getOutputStream socket)
                    in (.getInputStream socket)]
                (.flush out)
                {:socket socket
                 :out out :in in
                 :status :connected})
              (catch SocketException e
                (if (Thread/interrupted)
                  (println "Net: Connection attempt interrupted")
                  (println "Net: Connection attempt failed:"
                    (.getMessage e)))
                nil)
              (catch SocketTimeoutException e
                (println "Net: Socket timed out"))
              (catch Exception e
                (prn e)))]
           (send *conn
             (fn [{:as conn}]
               (if (identical? (:socket conn) socket)
                 (if new-conn
                   (do
                     (println "Net: Connection successful")
                     new-conn)
                   {:socket socket
                    :out nil :in nil
                    :status :failed})
                 conn))))))}))

(defn reset-conn! []
  (let [res (promise)]
    (send *conn
      (fn [conn]
        (if (= :connecting (:status conn))
          conn
          (try
            (let [conn' (do (close-conn conn)
                          (connect-conn))]
              (deliver res conn')
              conn')
            (catch Throwable e
              (deliver res nil)
              (throw e))))))
    @res))

(defn shutdown-hook []
  (send-off *conn close-conn))

(.addShutdownHook (Runtime/getRuntime)
  (Thread. #'shutdown-hook))

(defn send-bytes! [conn data]
  (let [{:keys [^OutputStream out]} conn]
    (when (nil? out)
      (throw (IOException. "Nil 'out'")))
    ; (println "send" data)
    (.write out
      (byte-array data))
    (.flush out)))

(defn read-all-bytes! [conn]
  (let [{:keys [^InputStream in]} conn]
    (when in
      (let [nbytes (.available in)]
        (when (< 0 nbytes)
          (let [ba (.readNBytes in nbytes)]
            ; (println (vec ba))
            ba))))))

; (defn read-response! ^"[B" [conn]
;   (let [{:keys [^InputStream in]} conn]
;     (when in
;       (let [res-ba (byte-array 8)
;             nbytes (.read in res-ba)]
;         (when (< 0 nbytes)
;           res-ba)))))


(comment
  (reset-conn!)
  
  )