(ns idp.robot.params
  (:require
    [io.github.humbleui.paint :as paint]))

(def dims
  {:width 200
   :length 200
   :centre-y 170
   :line-sensors-y 50
   :line-sensors-spacing 19
   :line-sensors-centre-spacing 19
   
   :wheel-diameter 105
   :wheel-spacing 200
   :max-rpm 18
   
   :ultrasonic-1
   {:pos {:x -70 :y -70}
    :angle 180}
   :ultrasonic-2
   {:pos {:x 70 :y -70}
    :angle 0}
   })

(def theme
  {:border-stroke (paint/stroke 0xFFf0f090 3)
   
   :line-sensor-fill (paint/fill 0xFFff6f20)
   :line-sensor-radius 5
   
   :ultrasonic-fill (paint/fill 0xFF4097d1)})

(defn get-line-sensor-pos [n]
  (let [{:keys [line-sensors-spacing centre-y
                line-sensors-centre-spacing
                length line-sensors-y]} dims
        centre-offset (/ line-sensors-centre-spacing 2)
        offset (case n
                 4 (- 0 line-sensors-spacing centre-offset)
                 3 (- centre-offset)
                 2 centre-offset
                 1 (+ line-sensors-spacing centre-offset))]
    {:x offset :y (- line-sensors-y centre-y)}))