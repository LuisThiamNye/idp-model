(ns idp.robot.state
  (:require
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
    [idp.board.geo :as board.geo]
    [io.github.humbleui.debug :as debug]))

(def *real (atom nil))

(defn reset-real! []
  (reset! *real
    {:velocity 0
     :angular-velocity 0
     :position (board.params/get-start-box-centre)
     :angle 270
     
     :line-sensor-1 false
     :line-sensor-2 false
     :line-sensor-3 false
     }))
(reset-real!)

(defn tick! [dt]
  (swap! *real
    (fn [{:keys [position angle velocity angular-velocity] :as state}]
      (let [dt (/ dt 1000)
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
                (update :y + (:y position))))]
        ; (prn (absolutise-robot-point
        ;                      (robot.params/get-line-sensor-pos 1)))
        (assoc state
          :line-sensor-1 (board.geo/point-on-line?
                           (absolutise-robot-point
                             (robot.params/get-line-sensor-pos 1)))
          :line-sensor-2 (board.geo/point-on-line?
                           (absolutise-robot-point
                             (robot.params/get-line-sensor-pos 2)))
          :line-sensor-3 (board.geo/point-on-line?
                           (absolutise-robot-point
                             (robot.params/get-line-sensor-pos 3)))
          :position position
          :angle angle)))))
