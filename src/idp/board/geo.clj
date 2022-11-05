(ns idp.board.geo
  (:require
    [idp.board.params :as params]))

(defn rect-ltrb [l t r b]
  {:left l :top t :right r :bottom b})

(defn rect-contains? [{:keys [left right top bottom]} {:keys [x y]}]
  (and
    (<= left x) (< x right)
    (<= top y) (< y bottom)))

(defn get-line-geo* []
  (let [{:keys [board-width board-height
             line-left-margin
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
             ramp-margin-bottom]} params/dims
     line-turn-outer-radius (+ line-turn-radius line-width)
     line-horiz-xr (- board-width (+ line-turn-outer-radius line-right-margin))
     line-bottom-yb (- board-height line-bottom-margin)]
    {:top-rect
     (rect-ltrb
       (+ line-left-margin line-turn-outer-radius)
       line-top-margin
       line-horiz-xr
       (+ line-top-margin line-width))
     :right-rect
     (rect-ltrb
       (- board-width line-right-margin line-width)
       (+ line-top-margin line-turn-outer-radius)
       (- board-width line-right-margin)
       (- board-height (+ line-bottom-margin line-turn-outer-radius)))
     :bottom-rect
     (rect-ltrb
       (+ line-left-margin line-turn-outer-radius)
       (- board-height line-bottom-margin line-width)
       line-horiz-xr
       line-bottom-yb)
     :left-top-rect
     (rect-ltrb
       line-left-margin
       (+ line-top-margin line-turn-outer-radius)
       (+ line-left-margin line-width)
       tunnel-y)
     :left-bottom-rect
     (rect-ltrb
       line-left-margin
       (+ tunnel-y tunnel-length)
       (+ line-left-margin line-width)
       (- board-height (+ line-bottom-margin line-turn-outer-radius)))}))

(let [*prev-rev (atom nil)
      *prev-ret (atom nil)]
  (defn get-line-geo []
    (if (= @*prev-rev clojure.lang.Var/rev)
      @*prev-ret
      (do (reset! *prev-rev clojure.lang.Var/rev)
        (reset! *prev-ret (get-line-geo*))))))

(defn point-on-line? [point]
  (let
    [{:keys [top-rect
             right-rect
             bottom-rect
             left-top-rect
             left-bottom-rect]} (get-line-geo)]
    (or
      (rect-contains? top-rect point)
      (rect-contains? right-rect point)
      (rect-contains? bottom-rect point)
      (rect-contains? left-top-rect point)
      (rect-contains? left-bottom-rect point))))