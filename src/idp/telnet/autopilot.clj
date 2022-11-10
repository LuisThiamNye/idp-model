(ns idp.telnet.autopilot
  (:require
    [io.github.humbleui.core :as hui]
    [idp.robot.brain.travel :as travel]
    [idp.telnet.api :as api]
    [idp.telnet :as net]
    ))

(def *mode (atom :travel))

(defn tick! [dt]
  ; (idp.robot.brain.travel/tick!)
  #_(robot.state/tick! dt)
  ; (println "tick " dt (:status @net/*conn))
  (Thread/sleep 50)
  (api/tick! dt)
  (when (= :connected (:status @net/*conn))
    (prn @api/*state)
    ; (prn @api/*input)
    
    (when (= :travel @*mode)
      #_(travel/tick!)))
  )

(def *stop-loop? (atom false))

(def *last-tick-time (atom 0))
(def *loop-dt (atom 100))

(def *loop-thread (atom nil))

(defn tick-recurring []
  (if @*stop-loop?
    (reset! *stop-loop? false)
    (let [dt @*loop-dt
          now (System/currentTimeMillis)
          del (max 0 (- (+ dt
                          @*last-tick-time) now))]
      ; (prn del)
      (reset! *last-tick-time now)
      (tick! dt)
      (hui/schedule #(tick-recurring)
        del))))

(defn start-loop! []
  (let [prev-thread ^Thread @*loop-thread
        _ (when prev-thread
            (.interrupt prev-thread))]
    (reset! *loop-thread
      (.start (Thread/ofVirtual)
        (fn []
          (loop [t (System/currentTimeMillis)]
            (let [t2 (System/currentTimeMillis)]
              (tick! (- t2 t))
              (if @*stop-loop?
                (reset! *stop-loop? false)
                (recur t2)))))))))

(start-loop!)

(comment
  
  (do
    (reset! *stop-loop? true)
    (swap! api/*input assoc
      :motor-1 0
      :motor-2 0)
    (api/tick! 0))
  
  (reset! *mode :travel)
  (reset! *mode :none)
  (swap! api/*input assoc
      :motor-1 0
      :motor-2 0)
  
  @*loop-thread
  
  (.interrupt @*loop-thread)
  
  
  
  )