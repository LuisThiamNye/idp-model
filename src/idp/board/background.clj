(ns idp.board.background
  (:require
    [idp.board.params :as params]
    [idp.board.geo :as board.geo]
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
             line-bottom-yb (- board-height line-bottom-margin)
             ->rect #(Rect. (:left %) (:top %) (:right %) (:bottom %))]
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
           
           ;; Lines
           (let [{:keys [top-rect
                         right-rect
                         bottom-rect
                         left-top-rect
                         left-bottom-rect
                         tl-turn-centre
                         tr-turn-centre
                         bl-turn-centre
                         br-turn-centre]} (board.geo/get-line-geo)
                 turn-rect (fn [{:keys [x y]} radius]
                             (.inflate (Rect. x y x y) radius))
                 draw-sector
                 (fn [^Canvas cnv rect start-deg sweep-deg ^Paint paint]
                   (.drawArc cnv
                     (:x rect) (:y rect)
                     (:right rect) (:bottom rect)
                     start-deg
                     sweep-deg
                     true ;; includeCentre
                     paint))]
             (.drawRect cnv (->rect top-rect) line-fill)
             (.drawRect cnv (->rect right-rect) line-fill)
             (.drawRect cnv (->rect bottom-rect) line-fill)
             (.drawRect cnv (->rect left-top-rect) line-fill)
             (.drawRect cnv (->rect left-bottom-rect) line-fill)
             ;; Turns
             (draw-sector cnv
               (turn-rect tl-turn-centre line-turn-outer-radius)
               180 90 line-fill)
             (draw-sector cnv
               (turn-rect tl-turn-centre line-turn-radius)
               180 90 board-bg)
             
             (draw-sector cnv
               (turn-rect tr-turn-centre line-turn-outer-radius)
               270 90 line-fill)
             (draw-sector cnv
               (turn-rect tr-turn-centre line-turn-radius)
               270 90 board-bg)
             
             (draw-sector cnv
               (turn-rect bl-turn-centre line-turn-outer-radius)
               90 90 line-fill)
             (draw-sector cnv
               (turn-rect bl-turn-centre line-turn-radius)
               90 90 board-bg)
             
             (draw-sector cnv
               (turn-rect br-turn-centre line-turn-outer-radius)
               0 90 line-fill)
             (draw-sector cnv
               (turn-rect br-turn-centre line-turn-radius)
               0 90 board-bg)
             
             )
           
           ;; Line Boxes
           (let [draw-box
                 (fn [outer-rect' inner-rect' path-rect' box-fill]
                   (let [outer-rect (->rect outer-rect')]
                     (.drawRect cnv outer-rect box-fill)
                     (.drawRect cnv (->rect inner-rect') board-bg)
                     (.drawRect cnv (->rect path-rect') line-fill)))
                 {:keys [red-box-outer-rect
                         start-box-outer-rect
                         green-box-outer-rect
                         red-box-inner-rect
                         start-box-inner-rect
                         green-box-inner-rect
                         red-box-path-rect
                         start-box-path-rect
                         green-box-path-rect]} (board.geo/get-line-geo)]
             (draw-box red-box-outer-rect red-box-inner-rect
               red-box-path-rect line-box-red-fill)
             (draw-box start-box-outer-rect start-box-inner-rect
               start-box-path-rect line-fill)
             (draw-box green-box-outer-rect green-box-inner-rect
               green-box-path-rect line-box-green-fill))
           
           ;; Collection lines
           (let [{:keys [collect-left-rect
                         collect-centre-rect
                         collect-right-rect]}
                 (board.geo/get-line-geo)
                 draw-rect
                 (fn [rect]
                   (.drawRect cnv
                     (Rect/makeLTRB (:left rect) (:top rect)
                       (:right rect) (:bottom rect))
                     line-fill))]
             (draw-rect collect-left-rect)
             (draw-rect collect-centre-rect)
             (draw-rect collect-right-rect))
           
           )))}))

(def ui-background
  (ui/dynamic ctx
    [ui-bg-canvas ui-bg-canvas
     theme params/theme]
    (ui/with-context
      {:theme theme}
      ui-bg-canvas)))