(ns idp.robot.params
  (:require
    [io.github.humbleui.paint :as paint]))

(def dims
  {:width 200
   :length 300
   :centre-y 150
   :line-sensors-y 50
   :line-sensors-spacing 30})

(def theme
  {:border-stroke (paint/stroke 0xFFf0f090 3)
   
   :line-sensor-fill (paint/fill 0xFFff6f20)
   :line-sensor-radius 5})

(defn get-line-sensor-pos [n]
  (let [{:keys [line-sensors-spacing
                length line-sensors-y]} dims
        offset (case n
                 1 (- line-sensors-spacing)
                 2 0
                 3 line-sensors-spacing)]
    {:x offset :y (- line-sensors-y (/ length 2))}))