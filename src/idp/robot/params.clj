(ns idp.robot.params
  (:require
    [io.github.humbleui.paint :as paint]))

(def dims
  {:width 230
   :length 250
   :centre-y 170
   :line-sensors-y 40
   :line-sensors-spacing 30
   :line-sensors-centre-spacing 19
   
   :wheel-diameter 100
   :wheel-spacing 300
   :max-rpm 18})

(def theme
  {:border-stroke (paint/stroke 0xFFf0f090 3)
   
   :line-sensor-fill (paint/fill 0xFFff6f20)
   :line-sensor-radius 5})

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