(ns idp.robot.graphic
  (:require
    [idp.robot.params :as params]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.core :as core]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.canvas :as canvas])
  (:import
    [io.github.humbleui.types IRect IPoint Rect]
    [io.github.humbleui.skija Canvas ImageFilter SaveLayerRec Paint]
    [java.lang AutoCloseable]))

(def ui-robot
  (ui/dynamic ctx
    [dims params/dims
     theme params/theme]
    (ui/with-context
     {:dims dims
      :theme theme}
     (ui/canvas
       {:on-paint
        (fn [ctx ^Canvas cnv size]
          (let [scale (* (:scale ctx) (:board-scale ctx))
                {:keys [width length centre-y
                        line-sensors-y line-sensors-spacing]} (:dims ctx)
                {:keys [border-stroke
                        line-sensor-fill
                        line-sensor-radius]} (:theme ctx)
                xmid (/ width 2)
                ymid centre-y
                {:keys [angle]
                 {robot-x :x
                  robot-y :y} :position} @(:*robot-real-state ctx)]
            (canvas/with-canvas
              (canvas/scale cnv scale)
              (canvas/translate cnv robot-x robot-y)
              (canvas/rotate cnv (- angle 270))
              (canvas/translate cnv (- xmid) (- ymid))
              (.drawRect cnv (Rect/makeWH width length) border-stroke)
              ;; line sensors
              (.drawCircle cnv xmid line-sensors-y line-sensor-radius line-sensor-fill)
              (.drawCircle cnv (+ xmid line-sensors-spacing) line-sensors-y line-sensor-radius line-sensor-fill)
              (.drawCircle cnv (- xmid line-sensors-spacing) line-sensors-y line-sensor-radius line-sensor-fill)
              ;; centre
              (.drawCircle cnv xmid ymid line-sensor-radius border-stroke)
              
              )))}))))

; (def ui-board-robot
;   (ui/dynamic ctx [ui-robot ui-robot]
;     ()))