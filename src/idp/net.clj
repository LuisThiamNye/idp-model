(ns idp.net
  "Primitives for connecting to and transmitting data
  to the Arduino."
  (:require
    [clojure.java.shell :as shell])
  (:import
    (java.net Socket InetSocketAddress SocketException
      SocketTimeoutException)
    (java.io
      PrintWriter BufferedReader InputStreamReader
      Reader IOException InputStream OutputStream)))

(def server-mac-address
  "MAC address of the Arduino"
  "84-cc-a8-2e-96-18")
(def ^Long socket-port 23)
(def socket-connect-timeout 2000)

(let [ip-pattern (re-pattern (str #"\s(\S+)\s+" server-mac-address))]
  (defn get-server-ip
    "Finds the IP address of the Arduino connected to the current
     Windows device via mobile hotspot."
    []
    (let [{:keys [out]} (shell/sh "arp" "-a")
          [_ ip] (re-find ip-pattern out)]
      ip)))

(def disabled-state
  {:status :disabled ;; :disabled, :connecting, :connected, :failed
   :*socket nil
   :port socket-port
   :hostname nil})
(def *state (atom disabled-state))

(defn close-socket-safely [^Socket socket]
  (when socket
    (try (.close socket)
      (catch IOException e
        (println "Net: Failed to close socket: "
          (.getMessage e)))))
  nil)

(defn close-connection!
  "Disables the client and closes any connected socket"
  []
  (let [[prev-state _]
        (swap-vals! *state
          (fn [_] disabled-state))]
    (some-> (:*socket prev-state)
      (send close-socket-safely))))

(defn shutdown-hook []
  (close-connection!))

(.addShutdownHook (Runtime/getRuntime)
  (Thread. #'shutdown-hook))

(defn socket-err-handler [_agt ^Throwable e]
  (println "Socket agent error:" (.getMessage e)))

(defn make-socket-agent [socket]
  (agent socket
    :error-mode :continue
    :error-handler socket-err-handler))

(defn connect-socket
  "Attempts to connect the socket, returning true if successful"
  [^String hostname ^Long port ^Socket socket]
  (try
    (println "Net: Connecting to " hostname ":" port)
    (.connect socket
      (InetSocketAddress. hostname port)
      socket-connect-timeout)
    (println "Net:    successful connection")
    true
    (catch SocketException e
      (if (Thread/interrupted)
        (println "Net: Connection attempt interrupted")
        (println "Net: Connection attempt failed:"
          (.getMessage e)))
      nil)
    (catch SocketTimeoutException _
      (println "Net: Socket timed out"))
    (catch Exception e
      (println "Net: Unexpected connection error:" e))))

(defn reset-connection!
  "Closes any existing connection and starts a new one.
  Returns a promise of any new successful connection state.
  If provided state is stale, promise yields nil."
  [prev-state]
  (let [socket (Socket.)
        *socket (make-socket-agent socket)
        state (assoc prev-state
                :status :connecting
                :*socket *socket)
        *ret (promise)]
    (if (compare-and-set! *state prev-state state)
      (let [*prev-socket (:*socket prev-state)
            _ (when *prev-socket
                (send *prev-socket close-socket-safely))
            ; hostname (:hostname state)
            hostname (get-server-ip)
            return-new-state
            (fn [state2]
              (compare-and-set! *state state state2)
              (deliver *ret state2))]
        (if (nil? hostname)
          (return-new-state
            (assoc state :status :failed))
         (let [connect!
               (fn []
                 (send-off *socket
                   (fn [socket]
                     (return-new-state
                       (if (connect-socket
                             hostname (:port state) socket)
                         (assoc state :status :connected)
                         (assoc state :status :failed)))
                     socket)))]
           ;; Only begin new connection after previous socket has been closed
           (if *prev-socket
             (send *prev-socket (fn [_] (connect!)))
             (connect!)))))
      ;; prev-state is stale; return nil
      (deliver *ret nil))
    *ret))

(defn send-bytes!
  "Writes and flushes a sequence of bytes on the output stream.
  May throw IOException"
  [^Socket socket data]
  (let [out (.getOutputStream socket)]
    ; (println "send" data)
    (.write out (byte-array data))
    (.flush out)))

(defn read-all-bytes!
  "Nil if no available data.
  Returns byte array of input.
  May throw IOException."
  [^Socket socket]
  (let [in (.getInputStream socket)]
    (let [nbytes (.available in)]
      (when (< 0 nbytes)
        (let [ba (.readNBytes in nbytes)]
          ; (println (vec ba))
          ba)))))
