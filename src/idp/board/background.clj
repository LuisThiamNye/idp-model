(ns idp.board.background
  (:require
    [idp.board.params :as params]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.core :as core]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.canvas :as canvas])
  (:import
    [io.github.humbleui.types IRect IPoint Rect]
    [io.github.humbleui.skija Canvas ImageFilter SaveLayerRec Paint]
    [java.lang AutoCloseable]))

; (core/deftype+ )

(def ui-bg-canvas
  (ui/canvas
    {:on-paint
     (fn [ctx ^Canvas cnv ^IPoint size]
       (let [{:keys [board-width board-height]} (:dims ctx)
             scale (* (:board-scale ctx) (:scale ctx))
             {:keys [line-left-margin
                     line-top-margin
                     line-right-margin
                     line-bottom-margin
                     line-turn-radius
                     line-width
                     line-left-boxes-path-spacing
                     line-right-boxes-path-spacing
                     line-left-box-path-x
                     line-box-length
                     line-box-path-length
                     line-collect-left-x
                     line-collect-left-spacing
                     line-collect-right-margin
                     line-collect-offset-length
                     line-collect-centre-length
                     tunnel-y
                     tunnel-width
                     tunnel-wall-thickness
                     tunnel-length
                     ramp-width
                     ramp-margin-top
                     ramp-margin-bottom]} (:dims ctx)
             line-turn-outer-radius (+ line-turn-radius line-width)
             {:keys [line-fill
                     line-box-red-fill
                     line-box-green-fill
                     board-bg
                     board-border-bg
                     tunnel-bg
                     barrier-bg
                     ramp-bg]} (:theme ctx)
             line-horiz-xr (- board-width (+ line-turn-outer-radius line-right-margin))
             turn-square-length (* 2 line-turn-outer-radius)
             turn-insquare-length (* 2 line-turn-radius)
             line-bottom-yb (- board-height line-bottom-margin)]
         (.drawRect cnv (Rect/makeWH (.toPoint size)) board-border-bg)
         (canvas/with-canvas cnv
           (canvas/scale cnv scale)
           (.drawRect cnv (Rect. 0 0 board-width board-height) board-bg)
           
           ;; Tunnel
           (.drawRect cnv (Rect/makeXYWH 0 tunnel-y
                            tunnel-wall-thickness tunnel-length)
            barrier-bg)
           (.drawRect cnv (Rect/makeXYWH (+ tunnel-wall-thickness tunnel-width) tunnel-y
                            tunnel-wall-thickness tunnel-length)
            barrier-bg)
           (.drawRect cnv (Rect/makeXYWH tunnel-wall-thickness tunnel-y
                            tunnel-width tunnel-length)
            tunnel-bg)
           ;; Ramp
           (.drawRect cnv (Rect/makeLTRB (- board-width ramp-width) ramp-margin-top
                            board-width (- board-height ramp-margin-bottom))
            ramp-bg)
           
           ;; top line
           (.drawRect cnv (Rect.
                           (+ line-left-margin line-turn-outer-radius)
                           line-top-margin
                           line-horiz-xr
                           (+ line-top-margin line-width))
            line-fill)
           ;; right line
           (.drawRect cnv (Rect.
                           (- board-width line-right-margin line-width)
                           (+ line-top-margin line-turn-outer-radius)
                           (- board-width line-right-margin)
                           (- board-height (+ line-bottom-margin line-turn-outer-radius)))
            line-fill)
           ;; bottom line
           (.drawRect cnv (Rect.
                            (+ line-left-margin line-turn-outer-radius)
                            (- board-height line-bottom-margin line-width)
                            line-horiz-xr
                            line-bottom-yb)
            line-fill)
           ;; left lines
           (.drawRect cnv (Rect.
                           line-left-margin
                           (+ line-top-margin line-turn-outer-radius)
                           (+ line-left-margin line-width)
                           tunnel-y)
            line-fill)
           (.drawRect cnv (Rect.
                           line-left-margin
                           (+ tunnel-y tunnel-length)
                           (+ line-left-margin line-width)
                           (- board-height (+ line-bottom-margin line-turn-outer-radius)))
            line-fill)
           
           ;; Turns
           (.drawArc cnv
             line-left-margin line-top-margin
             (+ line-left-margin turn-square-length)
             (+ line-top-margin turn-square-length)
             180 ;; start
             90 ;; sweep
             true ;; includeCentre
             line-fill)
           (let [l (+ line-left-margin line-width)
                 t (+ line-top-margin line-width)]
             (.drawArc cnv l t
              (+ l turn-insquare-length)
              (+ t turn-insquare-length)
              180 ;; start
              90 ;; sweep
              true ;; includeCentre
               board-bg))
           
           (.drawArc cnv
             (- board-width (+ line-right-margin turn-square-length))
             line-top-margin
             (- board-width line-left-margin)
             (+ line-top-margin turn-square-length)
             270 ;; start
             90 ;; sweep
             true ;; includeCentre
             line-fill)
           (let [r (- board-width (+ line-right-margin line-width))
                 t (+ line-top-margin line-width)]
             (.drawArc cnv
               (- r turn-insquare-length)
               t r
               (+ t turn-insquare-length)
               270 ;; start
               90 ;; sweep
               true ;; includeCentre
               board-bg))
           
           (.drawArc cnv
             (- board-width (+ line-right-margin turn-square-length))
             (- board-height (+ line-bottom-margin turn-square-length))
             (- board-width line-left-margin)
             (- board-height line-bottom-margin)
             0 ;; start
             90 ;; sweep
             true ;; includeCentre
             line-fill)
           (let [r (- board-width (+ line-right-margin line-width))
                 b (- board-height (+ line-bottom-margin line-width))]
             (.drawArc cnv
               (- r turn-insquare-length)
               (- b turn-insquare-length)
               r b
               0 ;; start
               90 ;; sweep
               true ;; includeCentre
               board-bg))
           
           (.drawArc cnv
             line-left-margin
             (- board-height (+ line-bottom-margin turn-square-length))
             (+ line-left-margin turn-square-length)
             (- board-height line-bottom-margin)
             90 ;; start
             90 ;; sweep
             true ;; includeCentre
             line-fill)
           (let [l (+ line-left-margin line-width)
                 b (- board-height (+ line-bottom-margin line-width))]
             (.drawArc cnv
               l
               (- b turn-insquare-length)
               (+ l turn-insquare-length)
               b
               90 ;; start
               90 ;; sweep
               true ;; includeCentre
               board-bg))
           
           ;; Line Boxes
           (let [shrink-outer
                 (fn [^Rect rect]
                   (.inflate rect (- line-width)))
                 draw-box
                 (fn [line-path-x box-fill]
                   (let [half-line-width (/ line-width 2)
                         outer-rect (Rect/makeXYWH
                                      (- (+ line-path-x half-line-width)
                                        (/ line-box-length 2))
                                      (+ line-bottom-yb line-box-path-length)
                                      line-box-length
                                      line-box-length)]
                     (.drawRect cnv outer-rect box-fill)
                     (.drawRect cnv (shrink-outer outer-rect) board-bg)
                     (.drawRect cnv (Rect/makeXYWH line-path-x line-bottom-yb
                                      line-width line-box-path-length)
                       line-fill)))]
             (draw-box line-left-box-path-x line-box-red-fill)
             (draw-box (+ line-left-box-path-x
                         line-left-boxes-path-spacing)
               line-fill)
             (draw-box (+ line-left-box-path-x
                         line-left-boxes-path-spacing
                         line-right-boxes-path-spacing)
               line-box-green-fill))
           
           ;; Collection lines
           (.drawRect cnv (Rect/makeXYWH line-collect-left-x (+ line-top-margin line-width)
                            line-width line-collect-offset-length)
             line-fill)
           (.drawRect cnv (Rect/makeXYWH (- board-width line-collect-right-margin line-width) (+ line-top-margin line-width)
                            line-width line-collect-offset-length)
             line-fill)
           (let [ymid (+ line-top-margin (/ line-width 2))]
             (.drawRect cnv (Rect/makeXYWH (+ line-collect-left-x line-collect-left-spacing)
                              (- ymid (/ line-collect-centre-length 2))
                              line-width line-collect-centre-length)
               line-fill))
           
           )))}))

(def ui-background
  (ui/dynamic ctx
    [ui-bg-canvas ui-bg-canvas
     theme params/theme]
    (ui/with-context
      {:theme theme}
      ui-bg-canvas)))