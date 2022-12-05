(ns idp.robot.graphic
  (:require
    [idp.robot.params :as params]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.canvas :as canvas])
  (:import
    (io.github.humbleui.types Rect)
    (io.github.humbleui.skija Canvas Path)))

(def ui-robot
  "Draws the graphical representation of the robot on the table,
  including translation and rotation."
  (ui/dynamic _
    [dims params/dims
     theme params/theme]
    (ui/with-context
     {:dims dims
      :theme theme}
     (ui/canvas
       {:on-paint
        (fn [ctx ^Canvas cnv _size]
          (let [scale (* (:scale ctx) (:board-scale ctx))
                {:keys [width length centre-y
                        ultrasonic-1 ultrasonic-2]} (:dims ctx)
                {:keys [border-stroke
                        ultrasonic-fill
                        line-sensor-fill
                        line-sensor-radius]} (:theme ctx)
                xmid (/ width 2)
                ymid centre-y
                {:keys [angle]
                 {robot-x :x
                  robot-y :y} :position
                 :as real-state} @(:*robot-real-state ctx)
                draw-line-sensor
                (fn [n]
                  (let [{:keys [x y]} (params/get-line-sensor-pos n)]
                    (.drawCircle cnv (+ x xmid) (+ y ymid)
                      line-sensor-radius line-sensor-fill)))
                ;; Draws triangles to represent the ultrasonic sensor positions
                ;; and orientations
                draw-ultrasonic
                (fn [{:keys [pos angle]}]
                  (let [x 0 y 0 l 7]
                    (canvas/with-canvas cnv
                      (.translate cnv (+ (:x pos) xmid) (+ (:y pos) ymid))
                      (.rotate cnv (- angle 90))
                      (.drawPath cnv
                        (doto (Path.)
                          (.addPoly
                            (float-array
                              [(+ x l) y
                               (- x l) (+ y l)
                               (- x l) (- y l)])
                            true))
                        ultrasonic-fill))))
                ;; Draws a straight line from the ultrasonic to the point on the
                ;; obstacle it is looking at
                draw-us-line
                (fn [us-key]
                  (let [{:keys [pos collision-pos]}
                        (us-key real-state)]
                    (when collision-pos
                      (.drawLine cnv
                        (:x pos) (:y pos)
                        (:x collision-pos) (:y collision-pos)
                        ultrasonic-fill))))]
            (canvas/with-canvas
              (canvas/scale cnv scale)
              
              (draw-us-line :ultrasonic-1)
              (draw-us-line :ultrasonic-2)
              
              (canvas/translate cnv robot-x robot-y)
              
              (canvas/rotate cnv (- angle 270))
              (canvas/translate cnv (- xmid) (- ymid))
              (.drawRect cnv (Rect/makeWH width length) border-stroke)

              (draw-line-sensor 1)
              (draw-line-sensor 2)
              (draw-line-sensor 3)
              (draw-line-sensor 4)
              
              (draw-ultrasonic ultrasonic-1)
              (draw-ultrasonic ultrasonic-2)
              
              ;; indicate the centre of rotation
              (.drawCircle cnv xmid ymid line-sensor-radius border-stroke)
              )))}))))