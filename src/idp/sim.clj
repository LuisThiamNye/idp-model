(ns idp.sim
  (:require
    [idp.robot.state :as state]
    [idp.loopthread :as loopth]
    [idp.robot.sim.tick :as robot.sim.tick])
  (:import
    (java.util Timer TimerTask)))

(defn tick! [_dt sim-dt]
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