(ns idp.robot.sim.client
  "Client implementation for communicating with simulated robot.
  Does not use sockets; network is simulated."
  (:require
    [taoensso.encore :as enc]
    [idp.robot.sim.server :as server]
    [idp.robot.client :as client]))

;; Simulate network latency and packet loss

(defn rand-send-latency []
  (+ 0 (rand-int 2)))
(defn rand-response-latency []
  (rand-send-latency))
(defn rand-request-drop? []
  ; (< (rand) 0.1)
  false
  )
(defn rand-response-drop? []
  (rand-request-drop?))

(defn process-queues
  "Propagates messages across the simulated network,
  making them available after their flight time defined
  by latency has been completed."
  [*state fromqk toqk]
  (let [state @*state
        fromq (fromqk state)
        pkg (peek fromq)]
    (when (and pkg
            (<= (+ (:latency pkg) (:timestamp pkg))
              (System/currentTimeMillis)))
      (let [msg (:message pkg)]
        (swap! *state #(-> %
                         (update fromqk pop)
                         (update toqk conj msg)))
        msg))))

(defrecord SimConnection [*state status])

(extend-type SimConnection client/Connection
  (-get-status [self] (:status self))
  
  (-get-response! [{:keys [*state]}]
    (process-queues *state :res-queue :ready-res-queue)
    (let [q (:ready-res-queue @*state)]
      (when (< 0 (count q))
        (swap! *state update :ready-res-queue pop)
        (peek q))))
  
  (-send-input! [{:keys [*state]} input]
    (when-not (rand-request-drop?)
      (swap! *state update :req-queue conj
        {:timestamp (System/currentTimeMillis)
         :latency (rand-send-latency)
         :message input})))
  )

(def initial-client-state
  {:status :connected
   :req-queue (enc/queue)
   :res-queue (enc/queue)
   :ready-res-queue (enc/queue)
   :ready-req-queue (enc/queue)})

(defrecord SimClient [*state])

(extend-type SimClient client/Client
  (-get-connection [{:keys [*state]}]
    (->SimConnection *state :connected))
  
  (-reset-connection! [{:keys [*state]} _conn]
    (swap! *state merge
      (assoc initial-client-state
        :status :connected))
    (->SimConnection *state :connected))
  )

(defrecord ServerConnection [*state])

(extend-type ServerConnection server/ServerConnection
  (-get-request [{:keys [*state]}]
    (process-queues *state :req-queue :ready-req-queue))
  
  (-send-response [{:keys [*state]} response]
   (when-not (rand-response-drop?)
     (swap! *state update :res-queue conj
       {:timestamp (System/currentTimeMillis)
        :latency (rand-response-latency)
        :message response})))
  )

(def *state (atom nil))
(reset! *state initial-client-state)
(def *client (atom nil))
(reset! *client (->SimClient *state))
(def server-connection (->ServerConnection *state))
(swap! server/*state assoc :conn server-connection)
