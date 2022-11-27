(ns idp.robot.autopilot
  (:require
    [idp.loopthread :as loopth]
    [idp.robot.brain.travel :as travel]
    [idp.net.api :as net.api]
    [idp.robot.client :as client]
    [idp.robot.brain.phase :as phase]
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
              (swap! (:*input robot) merge
                (travel/tick! *state readings @(:*input robot))))))))))

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

(comment
  (loopth/loop-running? *net-loop)
  (def active-robot robot.state/net-robot)
  (def active-robot robot.state/sim-robot)
  
  (hash @sim.client/*client)
  
  (swap! (:*state active-robot) assoc :next-phase-map
    {:exit-start :exit-start-turn
     :exit-start-turn :up-to-ramp
     :up-to-ramp :post-ramp-find-junction
     :post-ramp-find-junction :start-to-centre-block
     ; :exit-start-turn :start-to-centre-block-tunnel
     [:start-to-centre-block-tunnel] :detect-block
     :start-to-centre-block :detect-block
     :detect-block :tunnel-approach
     ; :detect-block :centre-block-180
     ; :centre-block-180 :tunnel-approach
     :tunnel-approach :through-tunnel
     :through-tunnel :up-to-box
     :up-to-box :box-approach-turn
     :box-approach-turn :box-approach-turn-spin
     :box-approach-turn-spin :box-approach-edge
     :box-approach-edge :backup-from-box
     
     [:backup-from-box {:go-home? true}] :up-to-home-entry
     :up-to-home-entry :align-to-home
     
     [:backup-from-box {:go-home? false}] :start-to-centre-block
     }
    )
  
  (swap! (:*state active-robot)
    phase/initialise-phase-on-state
    ; travel/exit-start
    travel/start-to-centre-block
    ; travel/tunnel-approach
    )
  (swap! (:*input active-robot) assoc
    :ultrasonic-active? true)
  
  
  (swap! (:*input active-robot) assoc
    :signal-block-density :low)
  (swap! (:*input active-robot) assoc
    :signal-block-density nil)
  
  (reset! (:*state active-robot)
    robot.state/initial-state)
  
  (swap! *net-loop-state
    assoc :delay 0)
  
  (swap! (:*input robot.state/net-robot) assoc
    :motor-1 255
    :motor-2 255)
  (swap! (:*input robot.state/net-robot) assoc
    :motor-1 0
    :motor-2 0)
  
  (swap! (:*state active-robot) assoc
    :competition-start-time (System/currentTimeMillis))
  
  (swap! (:*state active-robot) assoc
    :competition-start-time (System/currentTimeMillis))
  
  (swap! (:*state active-robot) assoc
    :density :low)
  
  
  )