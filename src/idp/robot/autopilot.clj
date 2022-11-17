(ns idp.robot.autopilot
  (:require
    [idp.loopthread :as loopth]
    [idp.robot.brain.travel :as travel]
    [idp.net.api :as net.api]
    [idp.robot.client :as client]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.state :as robot.state]))

(defn tick! [*state _dt]
  (let [state @*state
        speed (max 0.001 @(:*speed state))
        del (or (:delay state)
              (* 2 (/ @robot.state/*sim-dt 1000.)))
        sleep-ms (min 1000 (max 0 (int (/ del speed))))]
    (when (< 0 sleep-ms)
      (Thread/sleep sleep-ms))
    (let [robot (:robot state)
          *readings (:*readings robot)
          prev-readings @*readings
          t-presync (System/currentTimeMillis)]
      (client/sync! @(:*client state) (:robot state))
      (let [*state (:*state robot)
            t-postsync (System/currentTimeMillis)]
        (when-not (identical? prev-readings @*readings)
          ; (prn (= prev-readings @*readings))
          ; (prn (:time-received prev-readings))
          (swap! *state update :readings-history conj
            (swap! *readings assoc
              :time-received t-postsync
              ;; time since the last response
              :dt (* (- t-postsync
                       (if prev-readings
                         (:time-received prev-readings t-presync)
                         t-presync))
                    speed)))
          (when (:auto? @*state)
            (let [readings @*readings]
              (swap! (:*input robot) merge (travel/tick! *state readings)))))))))

(defn tick-fn [params]
  #(tick! params %))

(def *sim-loop-state
  (atom {:robot robot.state/sim-robot
         :delay 2
         :*speed robot.state/*sim-speed
         :*client sim.client/*client}))

(def *sim-loop
  (loopth/make-loop
    (tick-fn *sim-loop-state)))

(def *net-loop-state
  (atom {:robot robot.state/net-robot
         :delay 0
         :*speed (atom 1)
         :*client net.api/*client}))

(def *net-loop
  (loopth/make-loop
    (tick-fn *net-loop-state)))

(comment
  (loopth/loop-running? *net-loop)
  (def active-robot robot.state/net-robot)
  (def active-robot robot.state/sim-robot)
  
  (hash @sim.client/*client)
  
  (swap! (:*state active-robot) assoc :mode
    ; :simple-follow
    ; :tight-follow
    :enter-tunnel
    )
  
  (swap! (:*state active-robot) merge
    travel/state0-exit-start
    ; travel/state0-basic-follow
    ; travel/state0-tunnel-approach
    )
  
  (swap! (:*input active-robot) assoc
    :ultrasonic-active? true)
  
  (reset! (:*state active-robot)
    robot.state/initial-state)
  
  (swap! *net-loop-state
    assoc :delay 0)
  
  )