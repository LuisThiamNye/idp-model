(ns idp.board.params
  (:require
    [io.github.humbleui.paint :as paint])
  (:import
    (io.github.humbleui.types Point)))

;; spacing is independent of thickness (not the gap)
(def dims
  {:board-width 3000
   :board-height 3000
   
   :line-width 19
   :line-collect-left-x 800
   :line-collect-left-spacing 700
   :line-collect-right-margin 800
   :line-collect-offset-length 400
   :line-collect-centre-length 200
   :line-turn-radius 300
   
   :line-left-margin 200
   :line-top-margin 400
   :line-right-margin 200
   :line-bottom-margin 700
   
   :line-box-length 400 ;; outer length
   :line-box-path-length 200
   :line-left-box-path-x 650
   :line-left-boxes-path-spacing 800
   :line-right-boxes-path-spacing 800
   
   :tunnel-y 900
   :tunnel-length 1000
   :tunnel-width 400
   :tunnel-wall-thickness 20
   
   :ramp-width 600
   :ramp-margin-top 900
   :ramp-margin-bottom 1100
   })

(def theme
  {:line-fill (paint/fill 0xFFffffff)
   :line-box-green-fill (paint/fill 0xFFd6f9dd)
   :line-box-red-fill (paint/fill 0xFFf9d6d6)
   
   :board-bg (paint/fill 0xFF000000)
   :board-border-bg (paint/fill 0xFF303030)
   
   :tunnel-bg (paint/fill 0xFF072d05)
   :ramp-bg (paint/fill 0xFF072f38)
   
   :barrier-bg (paint/fill 0xFF505050)})

(defn get-start-box-centre []
  (let [{:keys [line-box-length line-bottom-margin board-height
                line-left-box-path-x line-left-boxes-path-spacing line-width
                line-box-path-length]} dims]
    {:x (+ line-left-box-path-x line-left-boxes-path-spacing (/ line-width 2))
     :y (+ (- board-height line-bottom-margin)
          line-box-path-length (/ line-box-length 2))}))