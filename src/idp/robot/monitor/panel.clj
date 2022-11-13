(ns idp.robot.monitor.panel
  (:require
    ; [clojure.pprint :as pprint]
    [clojure.string :as str]
    [zprint.core :as zprint]
    [idp.robot.params :as params]
    [idp.robot.state :as robot.state]
    [idp.board.geo :as board.geo]
    [idp.state :as state]
    [io.github.humbleui.app :as app]
    [io.github.humbleui.paint :as paint]
    [idp.common :as common :refer [multiline-label]]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.canvas :as canvas]
    [io.github.humbleui.window :as window]
    [io.github.humbleui.font :as font]
    [io.github.humbleui.font :as typeface]
    [idp.robot.autopilot :as autopilot]
    [idp.net.api :as net.api]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.client :as client]
    [idp.loopthread :as loopth])
  (:import
    [io.github.humbleui.types IRect IPoint Rect Point]
    [io.github.humbleui.skija Canvas ImageFilter SaveLayerRec Paint
     FontStyle Font Typeface FontWidth FontWeight FontSlant]
    [java.lang AutoCloseable]))

(def ui-line-sensor-dot
  (let [inactive-fill (paint/fill 0x4F000000)
        active-fill (paint/fill 0xFF40F040)]
    (ui/width 15
      (ui/height #(:width %)
        (ui/canvas
          {:on-paint
           (fn [ctx ^Canvas cnv size]
             (let [rx (/ (:width size) 2)
                   ry (/ (:height size) 2)]
               (.drawCircle cnv rx ry
                 (min rx ry)
                 (if (:line-sensor-triggered? ctx)
                   active-fill
                   inactive-fill))))})))))

(def ui-line-sensors
  (ui/dynamic ctx
    [ui-line-sensor-dot ui-line-sensor-dot
     {:keys [line-sensor-1
             line-sensor-2
             line-sensor-3
             line-sensor-4]} (:robot-readings ctx)
     {:keys [mouse-on-line?]} @state/*misc
     ui-ls (fn [ls]
             (ui/with-context
               {:line-sensor-triggered? ls}
               ui-line-sensor-dot))]
    (let [gap 5]
      (ui/row
       (ui-ls line-sensor-4)
       (ui/gap gap 0)
       (ui-ls line-sensor-3)
       (ui/gap gap 0)
       (ui-ls line-sensor-2)
       (ui/gap gap 0)
       (ui-ls line-sensor-1)
       (ui/gap (* 2.5 gap) 0)
       (ui/with-context
         {:line-sensor-triggered? mouse-on-line?}
         ui-line-sensor-dot)))))

(let [forward-fill (paint/fill 0xFF89dddd)
      backward-fill (paint/fill 0xFFddc489)
      border-stroke (paint/stroke 0xFF505050 3)]
  (def ui-motor
    (ui/rect border-stroke
      (ui/padding 1
        (ui/stack
          (ui/canvas
           {:on-paint
            (fn [ctx ^Canvas cnv size]
              (let [speed (:motor-speed ctx)
                    coeff (float (/ speed 255))
                    height (* (abs coeff) (:height size))]
                (.drawRect cnv
                  (Rect/makeXYWH
                    0 (if (neg? coeff) 0 (- (:height size) height))
                    (:width size) height)
                  (if (neg? coeff)
                    backward-fill
                    forward-fill))))})
          (ui/dynamic ctx [{:keys [motor-speed]} ctx]
            (ui/center
              (ui/label
                (str (unchecked-int
                       (Math/round (* 100 (float (/ motor-speed 255)))))
                  "%")))))))))

(def ui-motors
  (ui/height 100
    (ui/dynamic ctx [ui-motor ui-motor
                     motor-1 (:motor-1 (:robot-input ctx))
                     motor-2 (:motor-2 (:robot-input ctx))]
      (ui/row
        [:stretch 1 (ui/with-context {:motor-speed motor-1} ui-motor)]
        (ui/gap 10 0)
        [:stretch 1 (ui/with-context {:motor-speed motor-2} ui-motor)]))))

(let [fill (paint/fill 0xFF89dddd)
      nodata-fill (paint/fill 0xFFa5c6c6)
      border-stroke (paint/stroke 0xFF505050 3)]
  (defn ui-ultrasonic [us-key]
    (ui/rect border-stroke
      (ui/padding 1
        (ui/stack
          (ui/canvas
            {:on-paint
             (fn [ctx ^Canvas cnv ^IPoint size]
               (let [dist (us-key (:robot-readings ctx))
                     coeff (float (/ dist 255))
                     width (* (:width size) coeff)
                     nodata? (= 0 dist)]
                 (if nodata?
                   (.drawRect cnv (Rect/makeWH (.toPoint size)) nodata-fill)
                   (.drawRect cnv
                     (Rect/makeXYWH
                       (if (= :ultrasonic-1 us-key)
                         (- (:width size) width)
                         0) 0
                       width (:height size))
                     fill))))})
          (ui/dynamic ctx [dist (us-key (:robot-readings ctx))]
            (ui/center
              (ui/label (str dist " mm")))))))))

(def ui-ultrasonics
  (ui/dynamic _ [ui-ultrasonic ui-ultrasonic]
    (ui/height 50
      (ui/row
        [:stretch 1 (ui-ultrasonic :ultrasonic-1)]
        (ui/gap 10 0)
        [:stretch 1 (ui-ultrasonic :ultrasonic-2)]))))

(let [bg-fill (paint/fill 0xFFe0e0e0)
      active-fill (paint/fill 0xFF89dddd)
      nodata-fill (paint/fill 0xFFa5c6c6)
      px-per-t 0.02]
  (def ui-ultrasonics-graph
    (ui/width 100
      (ui/canvas
        {:on-paint
         (fn [ctx ^Canvas cnv ^IPoint size]
           (.drawRect cnv (Rect/makeWH (.toPoint size)) bg-fill)
           (let
             [history (:readings-history (:robot-state ctx))
              scale (:scale ctx)
              px-per-t (* scale px-per-t)
              gap (* scale 2)
              seg-width (- (float (/ (:width size) 2)) (/ gap 2))
              rect-height (:height size)
              max-dist 2800]
             (reduce
               (fn [y {:keys [ultrasonic-1 ultrasonic-2 dt]}]
                 (let [seg-height (* px-per-t dt)
                       y2 (+ y seg-height)
                       draw-bar
                       (fn [ultrasonic left? ]
                         (let [nodata? (= 0 ultrasonic)
                               width (* seg-width
                                       (if nodata?
                                         1
                                         (min 1 (/ ultrasonic max-dist))))]
                           (.drawRect cnv
                             (Rect/makeXYWH
                               (if left?
                                 (- seg-width width)
                                 (+ seg-width gap)) y
                               width seg-height)
                             (if nodata? nodata-fill active-fill))))]
                   (draw-bar ultrasonic-1 true)
                   (draw-bar ultrasonic-2 false)
                   (if (<= rect-height y2)
                     (reduced nil)
                     y2)))
               0
               (rseq history))))}))))

(let [bg-fill (paint/fill 0xFFe0e0e0)
      active-fill (paint/fill 0xFF78bc78)
      missed-active-fill (paint/fill 0xFFb1bc78)
      missed-inactive-fill (paint/fill 0xFFcce0cc)
      px-per-t 0.02]
  (def ui-line-sensors-graph
    (ui/width 50
      (ui/canvas
        {:on-paint
         (fn [ctx ^Canvas cnv ^IPoint size]
           (.drawRect cnv (Rect/makeWH (.toPoint size)) bg-fill)
           (let
             [history (:readings-history (:robot-state ctx))
              scale (:scale ctx)
              px-per-t (* scale px-per-t)
              end (count history)
              seg-width (float (/ (:width size) 4))
              rect-height (:height size)]
             (loop [i (unchecked-dec end)
                    y (unchecked-int 0)]
               (when (and (<= 0 i)
                       (<= y rect-height))
                 (let [{:keys [line-sensor-1
                               line-sensor-2
                               line-sensor-3
                               line-sensor-4
                               line-switches
                               dt]} (nth history i)
                       seg-height (* px-per-t dt)
                       y2 (unchecked-int (+ y seg-height))
                       draw-seg
                       (fn [n x]
                         (let [on? (case n
                                     0 line-sensor-1
                                     1 line-sensor-2
                                     2 line-sensor-3
                                     3 line-sensor-4)
                               nswitches (nth line-switches n)
                               mseg-height (/ seg-height (inc nswitches))]
                           (when on?
                             (.drawRect cnv
                               (Rect/makeXYWH x y seg-width seg-height)
                               active-fill))
                           (loop [i 1
                                  on? (not on?)]
                             (when (<= i nswitches)
                               (.drawRect cnv
                                 (Rect/makeXYWH x (+ y (* i mseg-height))
                                   seg-width mseg-height)
                                 (if on?
                                   missed-active-fill
                                   missed-inactive-fill))
                               (recur (inc i) (not on?))))))]
                     (draw-seg 3 0)
                     (draw-seg 2 seg-width)
                     (draw-seg 1 (* 2 seg-width))
                     (draw-seg 0 (* 3 seg-width))
                 (recur (unchecked-dec i) y2))))))}))))

(let [;bg-fill (paint/fill 0xFFf7f7f7)
      fill (paint/fill 0xFFced9dd)
      px-per-t 0.2
      seg-width 2]
  (def ui-latency-graph
    (ui/width 100
      (ui/stack
        (ui/canvas
         {:on-paint
          (fn [ctx ^Canvas cnv ^IPoint size]
            ;(.drawRect cnv (Rect/makeWH (.toPoint size)) bg-fill)
            (let
              [history (:readings-history (:robot-state ctx))
               scale (:scale ctx)
               px-per-t (* scale px-per-t)
               end (count history)
               seg-width (* scale seg-width)
               start (max 0 (int (- end (Math/ceil (/ (:width size) seg-width)))))
               rect-height (:height size)]
              (reduce
                (fn [x {:keys [dt]}]
                  (let [seg-height (* px-per-t dt)
                        x2 (+ x seg-width)]
                    (.drawRect cnv
                      (Rect/makeLTRB
                        x (- rect-height seg-height) x2 rect-height)
                      fill)
                    x2))
                (max 0 (- (:width size) (* (- end start) seg-width)))
                (subvec history start end))))})
        (ui/halign 1 
          (ui/valign 1
            (ui/padding 5
              (ui/dynamic ctx
                [{:keys [dt]} (peek (:readings-history
                                      (:robot-state ctx)))]
                (ui/label (str dt " ms"))))))))))

(def ui-client-controls
  (ui/dynamic ctx
    [{looping? :client-looping?
      :keys [client-loop robot client]} ctx
     robot-auto? (:auto? @(:*state robot))]
    (ui/row
     (let [*looping (atom looping?)]
       (add-watch *looping :checkbox
         (fn [_ _ looping? looping?']
           (remove-watch *looping :checkbox)
           (if looping?'
             (loopth/start-loop! client-loop)
             (loopth/stop-loop! client-loop))))
       (ui/checkbox
         *looping
         (ui/label "Loop")))
     (ui/gap 10 0)
     (let [*auto (atom robot-auto?)]
       (add-watch *auto :checkbox
         (fn [_ _ _ auto?]
           (when-not auto?
             (swap! (:*input robot) assoc
               :motor-1 0 :motor-2 0))
           (swap! (:*state robot) assoc :auto? auto?)))
       (ui/checkbox
         *auto
         (ui/label "Auto")))
     (ui/gap 5 0)
     (ui/button
       (fn [] (client/-reset-client! client))
       (ui/label "Reset")))))

(def ui-root
  (ui/mouse-listener
    {:on-move (fn [_] true)}
    (ui/stack
      (ui/canvas
        {:on-paint
         (fn [ctx _ _]
           (hui/schedule #(window/request-frame (:window ctx)) 20))})
      (ui/dynamic ctx
        [ui-line-sensors ui-line-sensors
         ui-motors ui-motors
         ui-ultrasonics ui-ultrasonics
         ui-ultrasonics-graph ui-ultrasonics-graph
         ui-line-sensors-graph ui-line-sensors-graph
         ui-client-controls ui-client-controls
         ui-latency-graph ui-latency-graph
         {:keys [scale]} ctx
         client-loop autopilot/*sim-loop
         robot robot.state/sim-robot
         readings @(:*readings robot)
         robot-state @(:*state robot)
         robot-input @(:*input robot)
         client @sim.client/*client
         looping? (loopth/loop-running? client-loop)]
        (ui/with-context
          {:robot-readings readings
           :robot-state robot-state
           :robot-input robot-input
           :robot-client client
           :client-looping? looping?
           :client-loop client-loop
           :robot robot
           :client client
           :font-ui (font/make-with-cap-height
                      common/code-typeface
                      (* scale 10))}
          (ui/row
            ui-line-sensors-graph
            [:stretch 1
             (ui/column
               (ui/padding 5
                 (ui/row
                   ui-line-sensors
                   (ui/gap 10 0)
                   ui-client-controls
                   (ui/gap 10 0)
                   (ui/label (client/-get-status client))))
               (ui/padding 5 ui-motors)
               (ui/padding 5 ui-ultrasonics)
               [:stretch 1
                (ui/padding 5
                  (let [fmt-data #(zprint.core/zprint-str %
                                    {:map {:nl-separator? true
                                           :justify? true
                                           :justify {:max-variance 20}}
                                     :width 60})
                        ;#(with-out-str (pprint/pprint %))
                        ]
                    (ui/column
                      (ui/dynamic ctx
                        [readings (:robot-readings ctx)]
                        (multiline-label
                          (fmt-data (dissoc readings
                                      :line-sensor-1
                                      :line-sensor-2
                                      :line-sensor-3
                                      :line-sensor-4
                                      :ultrasonic-1
                                      :ultrasonic-2
                                      :dt
                                      :time-received))))
                      (ui/gap 0 8)
                      (ui/dynamic ctx
                        [state (:robot-state ctx)]
                        (multiline-label
                          (fmt-data (dissoc state :readings-history))))
                      (ui/gap 0 8)
                      (ui/dynamic ctx
                        [input (:robot-input ctx)]
                        (multiline-label
                          (fmt-data (dissoc input :motor-1 :motor-2)))))))]
               (ui/height 100 ui-latency-graph))]
            ui-ultrasonics-graph))))))

(def app
  (ui/dynamic ctx [ui-root ui-root
                   wc common/with-context]
    (wc ui-root)))

(def *app (atom nil))
(reset! *app app)
(def *window (atom nil))

(defn open-window! []
  (reset! *window
    (ui/window
      {:title    "Client Monitor"
       :bg-color 0xFFFFFFFF
       :exit-on-close? false}
      *app)))

(comment
  (app/doui-async
    (open-window!))
  )