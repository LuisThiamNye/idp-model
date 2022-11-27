(ns idp.robot.sim.tick
  "Main entry point for handling each clock cycle of the simulation."
  (:require
    [idp.board.geo :as board.geo]
    [idp.robot.sim.server :as sim.server]
    [idp.robot.params :as robot.params]
    [idp.robot.state :refer [*real]]))

(defn principal-angle
  "Conforms angle to range [0, 360)"
  [angle]
  (let [angle (rem angle 360)]
    #_(cond
      (< angle -180) (+ 360 angle)
      (< 180 angle) (- angle 360)
      :else angle)
    (double (if (< angle 0) (+ 360 angle) angle))))

(defn ray-rect-collision-point
  "Gets closest point where a ray hits a solid rectangle exterior"
  [{:keys [x y] :as origin} angle {:keys [left right top bottom]}]
  (let [closest-rect-x (cond
                         (< x left)
                         (when (or (< 270 angle) (< angle 90))
                           left)
                         (< right x)
                         (when (and (< 90 angle) (< angle 270))
                           right)
                         :else :inside)
        closest-rect-y (cond
                         (< y top)
                         (when (and (< 0 angle) (< angle 180))
                           top)
                         (< bottom y)
                         (when (and (< 180 angle) (< angle 360))
                           bottom)
                         :else :inside)]
    (if (and (= :inside closest-rect-x)
          (= :inside closest-rect-y))
      ;; origin inside the rectangle
      origin
      (let [rads (* Math/PI (/ angle 180))
            dydx (Math/tan rads)
            vpoint (when (number? closest-rect-x)
                     (let [dx (- closest-rect-x x)
                           y2 (+ y (* dx dydx))]
                       (when (<= top y2 bottom)
                         {:x  closest-rect-x :y y2})))
            hpoint (when (number? closest-rect-y)
                     (let [dy (- closest-rect-y y)
                           x2 (+ x (/ dy dydx))]
                       (when (<= left x2 right)
                         {:x x2 :y  closest-rect-y})))]
        (or vpoint hpoint)))))

(defn ray-rect-interior-collision-point
  "Gets closest point where a ray hits the interior wall of
  a rectangular container"
  [{:keys [x y] :as _origin} angle {:keys [left right top bottom]}]
  (let [closest-rect-x (cond
                         (or (<= 270 angle) (< angle 90))
                         right
                         (and (<= 90 angle) (< angle 270))
                         left)
        closest-rect-y (cond
                         (and (<= 0 angle) (< angle 180))
                         bottom
                         (and (<= 180 angle) (< angle 360))
                         top)]
    (let [rads (* Math/PI (/ angle 180))
          dydx (Math/tan rads)
          vpoint (let [dx (- closest-rect-x x)
                       y2 (+ y (* dx dydx))]
                   (when (<= top y2 bottom)
                     {:x  closest-rect-x :y y2}))
          hpoint (let [dy (- closest-rect-y y)
                       x2 (+ x (/ dy dydx))]
                   (when (<= left x2 right)
                     {:x x2 :y  closest-rect-y}))]
      (or vpoint hpoint))))

(defn tick-physics! [dt-micros]
  (swap! *real
    (fn [{:keys [position angle velocity angular-velocity] :as state}]
      (let
        [dt (/ dt-micros 1000000)
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
           (if (board.geo/point-on-line?
                 (absolutise-robot-point
                   (robot.params/get-line-sensor-pos n)))
             :white :black))
         {:keys [tunnel-outer-wall-rect
                 tunnel-inner-wall-rect
                 board-rect]} (board.geo/get-line-geo)
         sub-points (fn [p1 p2]
                      {:x (- (:x p1) (:x p2))
                       :y (- (:y p1) (:y p2))})
         vec-abs (fn [{:keys [x y]}]
                   (Math/sqrt
                     (+ (Math/pow x 2)
                       (Math/pow y 2))))
         get-us-collision-data-for-rect
         (fn [pos us-angle rect in-or-ext]
           (let [collision-pos
                 ((case in-or-ext
                    :interior ray-rect-interior-collision-point
                    :exterior ray-rect-collision-point)
                  pos us-angle rect)]
             (when collision-pos
               {:collision-pos collision-pos
                :distance (vec-abs (sub-points pos collision-pos))})))
         get-us-data
         (fn [us-key]
           (let [us (us-key robot.params/dims)
                 pos (absolutise-robot-point (:pos us))
                 us-angle (principal-angle (+ angle (:angle us)))
                 data
                 (reduce
                   (fn [winner [rect in-or-ext]]
                     (let [data (get-us-collision-data-for-rect
                                  pos us-angle rect in-or-ext)]
                       (if (and data
                             (< (:distance data) (:distance winner)))
                         data winner)))
                   {:distance ##Inf}
                   [[tunnel-outer-wall-rect :exterior]
                    [tunnel-inner-wall-rect :exterior]
                    [board-rect :interior]])
                 data (let [{:keys [distance collision-pos]} data
                            in-range? (< 25 distance 2000)]
                        (assoc data
                          :distance (if in-range? distance 0)
                          :collision-pos (when in-range? collision-pos)))]
             (assoc data :pos pos)))]
        (assoc state
          :line-sensors (mapv get-line-sensor (range 1 5))
          :ultrasonic-1 (get-us-data :ultrasonic-1)
          :ultrasonic-2 (get-us-data :ultrasonic-2)
          :position position
          :angle (principal-angle angle))))))

(defn tick! [dt-micros]
  (tick-physics! dt-micros)
  (sim.server/tick!))
