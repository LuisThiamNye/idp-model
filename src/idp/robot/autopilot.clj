(ns idp.robot.autopilot
  "Stores the autopilot loops for both the simulated and real robots"
  (:require
    [idp.loopthread :as loopth]
    [idp.robot.brain.core :as brain]
    [idp.net.api :as net.api]
    [idp.robot.client :as client]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.state :as robot.state]))

(defn tick! [*loop-state _dt]
  (let [loop-state @*loop-state
        speed (max 0.001 @(:*speed loop-state))]
    ;; Possibly introduce a delay to throttle the autopilot frequency
    (let [del (or (:delay loop-state)
                (* 2 (/ @robot.state/*sim-dt 1000.)))
          sleep-ms (min 1000 (max 0 (int (/ del speed))))]
      (when (< 0 sleep-ms)
        (Thread/sleep sleep-ms)))
    
    (let [robot (:robot loop-state)
          {:keys [*state *readings]} robot
          prev-readings @*readings
          t-presync (System/currentTimeMillis)
          ;; Send request based on *input state and receive response
          _ (client/sync! @(:*client loop-state) robot)
          t-postsync (System/currentTimeMillis)]
      (when-not (identical? prev-readings @*readings)
        ;; Store response
        (swap! *state update :readings-history conj
          (swap! *readings assoc
            :time-received t-postsync
            ;; time since the last response
            :dt (* (- t-postsync
                     (if prev-readings
                       (:time-received prev-readings t-presync)
                       t-presync))
                  speed)))
        ;; Update requested robot input if autopilot enabled
        (when (:auto? @*state)
          (let [readings @*readings]
            (swap! (:*input robot) merge
              (brain/tick! *state readings @(:*input robot)))))))))

(defn tick-fn [params]
  #(tick! params %))

(def *sim-loop-state (atom nil))
(reset! *sim-loop-state
  {:robot robot.state/sim-robot
   :delay 20
   :*speed robot.state/*sim-speed
   :*client sim.client/*client})

(def *sim-loop
  (loopth/make-loop
    (tick-fn *sim-loop-state)))

(def *net-loop-state (atom nil))
(reset! *net-loop-state
  {:robot robot.state/net-robot
   :delay 0
   :*speed (atom 1)
   :*client net.api/*client})

(def *net-loop
  (loopth/make-loop
    (tick-fn *net-loop-state)))
