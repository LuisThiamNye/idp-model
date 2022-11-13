(ns idp.robot.autopilot
  (:require
    [idp.loopthread :as loopth]
    [idp.robot.brain.travel :as travel]
    [idp.net.api :as net.api]
    [idp.robot.client :as client]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.state :as robot.state]))

; (def *mode (atom :travel))

#_(defn tick! [dt]
  ; (robot.state/tick! dt)
  ; (println "tick " dt (:status @net/*conn))
  (Thread/sleep 50)
  (api/tick! dt)
  (when (= :connected (:status @net/*conn))
    ; (prn (select-keys @api/*state
    ;        [:ultrasonic-1 :ultrasonic-2]))
    (println
      (format "%4d %4d"
        (:ultrasonic-1 @api/*state)
        (:ultrasonic-2 @api/*state)))
    ; (prn @api/*input)
    #_
    (let [{:keys [line-sensor-1
                  line-sensor-2
                  line-sensor-3
                  line-sensor-4]} @api/*state]
      (prn [line-sensor-1
            line-sensor-2
            line-sensor-3
            line-sensor-4]))
    (when (= :travel @*mode)
      (travel/tick!)))
  )

(defn tick! [params dt]
  ; (println "tick")
  (Thread/sleep 20)
  (let [robot (:robot params)
        *readings (:*readings robot)
        prev-readings @*readings
        t-presync (System/currentTimeMillis)]
    (client/sync! @(:*client params) (:robot params))
    (let [*state (:*state robot)
          t-postsync (System/currentTimeMillis)]
      (when-not (identical? prev-readings @*readings)
        ; (prn (= prev-readings @*readings))
        ; (prn (:time-received prev-readings))
        (swap! *state update :readings-history conj
          (swap! *readings assoc
            :time-received t-postsync
            ;; time sine the last response
            :dt (- t-postsync
                  (if prev-readings
                    (:time-received prev-readings)
                    t-presync))))
        (when (:auto? @*state)
          (let [readings @*readings]
            (swap! (:*input robot) merge (travel/tick! *state readings))))))))

(defn tick-fn [params]
  #(tick! params %))

(def *sim-loop
  (loopth/make-loop
    (tick-fn
      {:robot robot.state/sim-robot
       :*client sim.client/*client})))

(def *net-loop
  (loopth/make-loop
    (tick-fn
      {:robot robot.state/net-robot
       :*client net.api/*client})))

(comment
  (loopth/start-loop! *sim-loop)
  (loopth/stop-loop! *sim-loop)
  (loopth/loop-running? *sim-loop)
  
  (hash @sim.client/*client)
  
  (swap! travel/*state assoc :mode
    ; :simple-follow
    ; :tight-follow
    :enter-tunnel
    )
  
  (swap! api/*input assoc
    :ultrasonic-active? true)
  
  (reset! (:*state robot.state/sim-robot)
    {:auto? true
     :mode :exit-start
     :readings-history []
     :over-horiz? false
     :n-horiz-found 0
     :deviation :none})
  
  
  
  )