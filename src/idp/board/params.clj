(ns idp.board.params
  (:require
    [io.github.humbleui.paint :as paint])
  (:import
    (io.github.humbleui.types Point)))

;; spacing is independent of thickness (not the gap)
(def dims
  {:board-width (/ (+ 2400 2410) 2)
   :board-height 2405
   
   :line-width 19
   :line-collect-left-x 789
   :line-collect-left-spacing 400
   :line-collect-right-margin 791
   :line-collect-offset-length 183 ; length of protrusion
   :line-collect-centre-length 195 ; end to end
   :line-turn-radius 130 ;; approx
   
   :line-left-margin 175
   :line-top-margin 290
   :line-right-margin 190
   :line-bottom-margin 540
   
   :line-box-length 350 ;; outer length
   :line-box-path-length 165
   :line-left-box-path-x 492
   :line-left-boxes-path-spacing 696
   :line-right-boxes-path-spacing 693
   
   :tunnel-y 845
   :tunnel-length 500
   :tunnel-width 345
   :tunnel-wall-thickness 13
   
   :ramp-width 400
   :ramp-margin-top 688
   :ramp-margin-bottom 895
   :ramp-slope-length 160 ;; horizontal length
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