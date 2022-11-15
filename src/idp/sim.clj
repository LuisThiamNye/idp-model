(ns idp.sim
  (:require
    [idp.robot.state :as state]
    [idp.loopthread :as loopth]
    [idp.robot.sim.tick :as robot.sim.tick])
  (:import
    (java.util Timer TimerTask)
    (java.time Duration)))

; (def *nanos (atom (System/nanoTime)))

#_(defn tick! [dt]
  (let [t-prev @*nanos
        t (System/nanoTime)
        t-prev (min t t-prev)
        sim-dt @state/*sim-dt
        t-target (+ (/ t-prev 1000) sim-dt)
        speed @state/*sim-speed]
    (when (pos? speed)
      (robot.sim.tick/tick! (* speed sim-dt)))
    (let
      [delay-micros (- t-target (/ (System/nanoTime) 1000.))]
      (prn (/ (- t t-prev) 1000.) delay-micros)
      (if (pos? delay-micros)
        (do
          (Thread/sleep (int (Math/round (/ delay-micros 1000))))
          (reset! *nanos t))
        (reset! *nanos (- t (* 1000 delay-micros)))))))

(defn tick! [dt sim-dt]
  (robot.sim.tick/tick! sim-dt))

(def *task (atom nil))

(defn stop! []
  (let [prev-task ^TimerTask @*task]
    (when prev-task
      (.cancel prev-task))
    (reset! *task nil)))

(defn start! []
  (let [sim-dt @state/*sim-dt
        dt (long (Math/ceil
                   (/ sim-dt
                     (max 0.0001 @state/*sim-speed)
                     1000.)))
        task (proxy [TimerTask] []
               (run []
                 (try (tick! dt sim-dt)
                   (catch Throwable e
                     (stop!)
                     (throw e)))))]
    (stop!)
    (when (pos? dt)
      (reset! *task task)
      (.scheduleAtFixedRate (Timer.) task 0 dt))))

(defn sim-running? []
  (some? @*task))

; (def *loop (loopth/make-loop #'tick!))
(comment
  (loopth/start-loop! *loop)
  (loopth/stop-loop! *loop)
  )