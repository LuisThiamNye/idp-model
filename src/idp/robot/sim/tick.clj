(ns idp.robot.sim.tick
  (:require
    [idp.board.geo :as board.geo]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.params :as robot.params]
    [idp.robot.state :refer [*real]]))

(defn principal-angle [angle]
  (let [angle (rem angle 360)]
    #_(cond
      (< angle -180) (+ 360 angle)
      (< 180 angle) (- angle 360)
      :else angle)
    (double (if (< angle 0) (+ 360 angle) angle))))

(defn tick! [dt-micros]
  (swap! *real
    (fn [{:keys [position angle velocity angular-velocity] :as state}]
      (let [dt (/ dt-micros 1000000)
            angle' (* (/ angle 180) Math/PI)
            dx (* dt velocity (Math/cos angle'))
            dy (* dt velocity (Math/sin angle'))
            position (-> position
                       (update :x + dx)
                       (update :y + dy))
            dangle (* angular-velocity dt)
            angle (+ angle dangle)
            rot-2d (fn [deg {:keys [x y]}]
                     (let [rad (* (/ deg 180) Math/PI)]
                       {:x (+ (* x (Math/cos rad))
                             (* y (- (Math/sin rad))))
                        :y (+ (* x (Math/sin rad))
                             (* y (Math/cos rad)))}))
            absolutise-robot-point
            (fn [pos]
              (-> (rot-2d (- angle 270) pos)
                (update :x + (:x position))
                (update :y + (:y position))))
            get-line-sensor
            (fn [n]
              (board.geo/point-on-line?
                (absolutise-robot-point
                  (robot.params/get-line-sensor-pos n))))]
        (assoc state
          :line-sensor-1 (get-line-sensor 1)
          :line-sensor-2 (get-line-sensor 2)
          :line-sensor-3 (get-line-sensor 3)
          :line-sensor-4 (get-line-sensor 4)
          :ultrasonic-1 0
          :ultrasonic-2 200
          :position position
          :angle (principal-angle angle)))))
  (sim.client/tick!))
