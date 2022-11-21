(ns idp.board.geo
  "Helper functions to obtain data about the board geometry,
  derived from parameters in `idp.board.params`"
  (:require
    [idp.board.params :as params]))

(defn rect-ltrb [l t r b]
  {:left l :top t :right r :bottom b})

(defn rect-xywh [x y w h]
  (rect-ltrb x y (+ x w) (+ y h)))

(defn rect-grow [rect d]
  (-> rect
    (update :left - d)
    (update :top - d)
    (update :right + d)
    (update :bottom + d)))

(defn rect-contains? [{:keys [left right top bottom]} {:keys [x y]}]
  (and
    (<= left x) (< x right)
    (<= top y) (< y bottom)))

(defn get-line-geo* []
  (let
    [{:keys [board-width board-height
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
     line-bottom-yb (- board-height line-bottom-margin)
     half-line-width (/ line-width 2)
     box-outer-rect
     (fn [line-path-x]
       (rect-xywh
         (- (+ line-path-x half-line-width)
           (/ line-box-length 2))
         (+ line-bottom-yb line-box-path-length)
         line-box-length
         line-box-length))
     shrink-outer-box
     (fn [rect] (rect-grow rect (- line-width)))
     green-box-outer-rect
     (box-outer-rect line-left-box-path-x)
     start-box-outer-rect
     (box-outer-rect (+ line-left-box-path-x
                       line-left-boxes-path-spacing))
     red-box-outer-rect
     (box-outer-rect (+ line-left-box-path-x
                       line-left-boxes-path-spacing
                       line-right-boxes-path-spacing))]
    {:board-rect (rect-xywh 0 0 board-width board-height)
     
     :top-rect
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
       (- board-height (+ line-bottom-margin line-turn-outer-radius)))
     
     :tl-turn-centre
     {:x (+ line-left-margin line-turn-outer-radius)
      :y (+ line-top-margin line-turn-outer-radius)}
     :tr-turn-centre
     {:x (- board-width (+ line-right-margin line-turn-outer-radius))
      :y (+ line-top-margin line-turn-outer-radius)}
     :bl-turn-centre
     {:x (+ line-left-margin line-turn-outer-radius)
      :y (- board-height (+ line-bottom-margin line-turn-outer-radius))}
     :br-turn-centre
     {:x (- board-width (+ line-right-margin line-turn-outer-radius))
      :y (- board-height (+ line-bottom-margin line-turn-outer-radius))}
     
     :collect-left-rect
     (rect-xywh line-collect-left-x (+ line-top-margin line-width)
       line-width line-collect-offset-length)
     :collect-centre-rect
     (rect-xywh (- board-width line-collect-right-margin line-width) (+ line-top-margin line-width)
       line-width line-collect-offset-length)
     :collect-right-rect
     (let [ymid (+ line-top-margin (/ line-width 2))]
       (rect-xywh (+ line-collect-left-x line-collect-left-spacing)
         (- ymid (/ line-collect-centre-length 2))
         line-width line-collect-centre-length))
     
     :red-box-path-rect
     (rect-xywh line-left-box-path-x
       line-bottom-yb
       line-width line-box-path-length)
     :start-box-path-rect
     (rect-xywh (+ line-left-box-path-x
                  line-left-boxes-path-spacing)
       line-bottom-yb
       line-width line-box-path-length)
     :green-box-path-rect
     (rect-xywh (+ line-left-box-path-x
                  line-left-boxes-path-spacing
                  line-right-boxes-path-spacing)
       line-bottom-yb
       line-width line-box-path-length)
     
     :red-box-outer-rect red-box-outer-rect
     :start-box-outer-rect start-box-outer-rect
     :green-box-outer-rect green-box-outer-rect
     :red-box-inner-rect (shrink-outer-box red-box-outer-rect)
     :start-box-inner-rect (shrink-outer-box start-box-outer-rect)
     :green-box-inner-rect (shrink-outer-box green-box-outer-rect)
     
     :tunnel-outer-wall-rect
     (rect-xywh
       (+ tunnel-wall-thickness tunnel-width) tunnel-y
       tunnel-wall-thickness tunnel-length)
     :tunnel-inner-wall-rect
     (rect-xywh 0 tunnel-y
       tunnel-wall-thickness tunnel-length)}))

;; (Memoised)
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
             left-bottom-rect
             collect-left-rect
             collect-centre-rect
             collect-right-rect
             red-box-path-rect
             start-box-path-rect
             green-box-path-rect
             tl-turn-centre
             tr-turn-centre
             bl-turn-centre
             br-turn-centre
             red-box-outer-rect
             start-box-outer-rect
             green-box-outer-rect
             red-box-inner-rect
             start-box-inner-rect
             green-box-inner-rect]} (get-line-geo)
     {:keys [line-turn-radius
             line-width]} params/dims
     line-turn-outer-radius (+ line-turn-radius line-width)
     dist-between (fn [p1 p2]
                    (Math/sqrt
                      (+ (Math/pow (- (:x p1) (:x p2)) 2)
                        (Math/pow (- (:y p1) (:y p2)) 2))))
     within-turn?
     (fn [centre]
       (let [d (dist-between centre point)]
         (and (<= line-turn-radius d)
           (<= d line-turn-outer-radius))))]
    (or
      ;; Straight Lines
      (rect-contains? top-rect point)
      (rect-contains? right-rect point)
      (rect-contains? bottom-rect point)
      (rect-contains? left-top-rect point)
      (rect-contains? left-bottom-rect point)
      (rect-contains? collect-left-rect point)
      (rect-contains? collect-centre-rect point)
      (rect-contains? collect-right-rect point)
      (rect-contains? red-box-path-rect point)
      (rect-contains? start-box-path-rect point)
      (rect-contains? green-box-path-rect point)
      ;; Turns
      (let [{:keys [x y] :as centre} tl-turn-centre]
        (and (rect-contains?
               (rect-ltrb
                 (- x line-turn-outer-radius)
                 (- y line-turn-outer-radius)
                 x y)
               point)
          (within-turn? centre)))
      (let [{:keys [x y] :as centre} tr-turn-centre]
        (and (rect-contains?
               (rect-ltrb
                 x
                 (- y line-turn-outer-radius)
                 (+ x line-turn-outer-radius)
                 y)
               point)
          (within-turn? centre)))
      (let [{:keys [x y] :as centre} bl-turn-centre]
        (and (rect-contains?
               (rect-ltrb
                 (- x line-turn-outer-radius)
                 y x
                 (+ y line-turn-outer-radius))
               point)
          (within-turn? centre)))
      (let [{:keys [x y] :as centre} br-turn-centre]
        (and (rect-contains?
               (rect-ltrb
                 x y
                 (+ x line-turn-outer-radius)
                 (+ y line-turn-outer-radius))
               point)
          (within-turn? centre)))
      ;; Boxes
      (and (rect-contains? red-box-outer-rect point)
        (not (rect-contains? red-box-inner-rect point)))
      (and (rect-contains? start-box-outer-rect point)
        (not (rect-contains? start-box-inner-rect point)))
      (and (rect-contains? green-box-outer-rect point)
        (not (rect-contains? green-box-inner-rect point))))))