(ns idp.robot.params
  (:require
    [io.github.humbleui.paint :as paint]))

(def dims
  "Important dimensions of the robot"
  (let [axle-y 170
        axle-to-ls 130]
    {:width 200
     :length 200
     :centre-y axle-y
     :line-sensors-y (- axle-y axle-to-ls)
     :line-sensors-spacing 19
     :line-sensors-centre-spacing 19
     
     :wheel-diameter 105
     :wheel-spacing 205
     :max-rpm 18
     
     :ultrasonic-1
     {:pos {:x -70 :y -70}
      :angle 270}
     :ultrasonic-2
     {:pos {:x 0 :y 30}
      :angle 180}
     }))

(def theme
  "Theme used to display the simulated robot"
  {:border-stroke (paint/stroke 0xFFf0f090 3)
   
   :line-sensor-fill (paint/fill 0xFFff6f20)
   :line-sensor-radius 5
   
   :ultrasonic-fill (paint/fill 0xFF4097d1)})

(defn get-line-sensor-pos
  "Returns the position of a line sensor relative
  to the centre of rotation of the robot,
  where the robot is at a 270° angle (pointed upwards)."
  [n]
  (let [{:keys [line-sensors-spacing centre-y
                line-sensors-centre-spacing
                line-sensors-y]} dims
        centre-offset (/ line-sensors-centre-spacing 2)
        offset (case (unchecked-int n)
                 4 (- 0 line-sensors-spacing centre-offset)
                 3 (- centre-offset)
                 2 centre-offset
                 1 (+ line-sensors-spacing centre-offset))]
    {:x offset :y (- line-sensors-y centre-y)}))