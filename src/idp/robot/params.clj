(ns idp.robot.params
  (:require
    [io.github.humbleui.paint :as paint]))

(def dims
  {:width 180
   :length 230
   :centre-y 170
   :line-sensors-y 170; 40
   :line-sensors-spacing 30})

(def theme
  {:border-stroke (paint/stroke 0xFFf0f090 3)
   
   :line-sensor-fill (paint/fill 0xFFff6f20)
   :line-sensor-radius 5})

(defn get-line-sensor-pos [n]
  (let [{:keys [line-sensors-spacing centre-y
                length line-sensors-y]} dims
        offset (case n
                 1 (- line-sensors-spacing)
                 2 0
                 3 line-sensors-spacing)]
    {:x offset :y (- line-sensors-y centre-y)}))