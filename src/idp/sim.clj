(ns idp.sim
  (:require
    [idp.loopthread :as loopth]
    [idp.robot.sim.tick :as robot.sim.tick]))

(defn tick! [dt]
  (Thread/sleep 10)
  (robot.sim.tick/tick! dt))

(def *loop (loopth/make-loop #'tick!))
(comment
  (loopth/start-loop! *loop)
  (loopth/stop-loop! *loop)
  )