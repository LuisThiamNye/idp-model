(ns idp.robot.monitor.panel
  "Visualisations of the robot sensor data and connection status."
  (:require
    [clojure.string :as str]
    [zprint.core :as zprint]
    [idp.robot.params :as params]
    [idp.robot.state :as robot.state]
    [idp.board.geo :as board.geo]
    [idp.state :as state]
    [idp.robot.brain.phase :as phase]
    [io.github.humbleui.app :as app]
    [io.github.humbleui.paint :as paint]
    [idp.common :as common :refer [multiline-label
                                   action-checkbox]]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.canvas :as canvas]
    [io.github.humbleui.window :as window]
    [io.github.humbleui.font :as font]
    [idp.robot.autopilot :as autopilot]
    [idp.net.api :as net.api]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.client :as client]
    [idp.loopthread :as loopth])
  (:import
    (io.github.humbleui.jwm Window)
    [io.github.humbleui.types IRect IPoint Rect Point]
    [io.github.humbleui.skija Canvas Paint
     FontStyle Font Typeface FontWidth FontWeight FontSlant]))

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
                 (if (= :white (:line-sensor-reading ctx))
                   active-fill
                   inactive-fill))))})))))

(def ui-line-sensors
  (ui/dynamic ctx
    [ui-line-sensor-dot ui-line-sensor-dot
     line-sensors (:line-sensors
                    (:robot-readings ctx))
     ; {:keys [mouse-on-line?]} @state/*misc
     ui-ls (fn [ls]
             (ui/with-context
               {:line-sensor-reading ls}
               ui-line-sensor-dot))]
    (let [gap 5]
      (ui/row
        (eduction
          (map ui-ls)
          (interpose (ui/gap gap 0))
          line-sensors)
       #_#_(ui/gap (* 2.5 gap) 0)
       (ui/with-context
         {:line-sensor-triggered? mouse-on-line?}
         ui-line-sensor-dot)))))

(def ui-rect-meter
  (let [border-stroke (paint/stroke 0xFF505050 3)
        fallback-fill (paint/fill 0xFF89dddd)
        fallback-nodata-fill (paint/fill 0xFFa5c6c6)]
    (ui/dynamic _ []
      (ui/rect border-stroke
        (ui/padding 1
          (ui/canvas
            {:on-paint
             (fn [ctx ^Canvas cnv ^IPoint size]
               (let [coeff (:meter-value ctx)]
                 (if (nil? coeff)
                   (let [no-data-fill (:no-data-fill ctx fallback-nodata-fill)]
                     (.drawRect cnv (Rect/makeWH (.toPoint size)) no-data-fill))
                   (let [zero-location (:meter-zero ctx :left)
                         signed? (= :h-mid zero-location)
                         full-width (cond-> (:width size) signed? (/ 2))
                         width (* full-width (float coeff))
                         fill (:fill ctx fallback-fill)
                         height (:height size)
                         loffset (if (= :left zero-location)
                                   0
                                   full-width)]
                     (.drawRect cnv
                       (if (neg? coeff)
                         (Rect/makeLTRB (+ loffset width) 0 loffset height)
                         (Rect/makeXYWH loffset 0 width height))
                       fill)))))}))))))

(def ui-motor
  (let [forward-fill (paint/fill 0xFFc3ddf7)
        backward-fill (paint/fill 0xFFf7e7c0)
        border-stroke (paint/stroke 0xFF505050 3)]
    (ui/stack
      (ui/dynamic ctx [ui-rect-meter ui-rect-meter
                       speed (:motor-speed ctx)]
        (ui/with-context
          {:meter-value (float (/ speed 255))
           :meter-zero :h-mid
           :fill (if (neg? speed)
                   backward-fill
                   forward-fill)}
          ui-rect-meter))
      (ui/dynamic ctx [{:keys [motor-speed]} ctx]
        (ui/center
          (ui/label
            (str (unchecked-int
                   (Math/round (* 100 (float (/ motor-speed 255)))))
              "%")))))))

(def ui-motors
  (ui/height 40
    (ui/dynamic ctx [ui-motor ui-motor
                     motor-1 (:motor-1 (:robot-input ctx))
                     motor-2 (:motor-2 (:robot-input ctx))]
      (ui/column
        [:stretch 1 (ui/with-context {:motor-speed motor-1} ui-motor)]
        [:stretch 1 (ui/with-context {:motor-speed motor-2} ui-motor)]))))

(defn ui-ultrasonic [us-key]
  (let [max-dist 800]
    (ui/dynamic _ [ui-rect-meter ui-rect-meter]
      (ui/stack
        (ui/dynamic ctx [dist (or (us-key (:robot-readings ctx)) 0)]
          (let [coeff (float (/ dist max-dist))]
            (ui/with-context
              {:meter-value coeff}
              ui-rect-meter)))
        (ui/dynamic ctx [dist (us-key (:robot-readings ctx))]
          (ui/valign 0.5
            (ui/label (str (format " %.1f" (float dist)) " mm"))))))))

(def ui-ultrasonics
  (ui/dynamic _ [ui-ultrasonic ui-ultrasonic]
    (ui/height 40
      (ui/column
        [:stretch 1 (ui-ultrasonic :ultrasonic-1)]
        ; (ui/gap 10 0)
        [:stretch 1 (ui-ultrasonic :ultrasonic-2)]))))

(def ui-ultrasonics-graph
  (let [bg-fill (paint/fill 0xFFe0e0e0)
        active-fill (paint/fill 0xFF89dddd)
        nodata-fill (paint/fill 0xFFa5c6c6)
        px-per-t 0.04
        max-dist 1400]
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
              seg-width (- (float (/ (:width size) 2))
                          (/ gap 2))
              rect-height (:height size)]
             (reduce
               (fn [y {:keys [ultrasonic-1 ultrasonic-2 dt]}]
                 (let [seg-height (min (* px-per-t dt) rect-height)
                       y2 (+ y seg-height)
                       draw-bar
                       (fn [ultrasonic left?]
                         (let [nodata? (= 0 ultrasonic)
                               coeff (if nodata?
                                       1
                                       (min 1 (float (/ ultrasonic max-dist))))
                               width (* coeff seg-width)]
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

(def ui-line-sensors-graph
  (let [bg-fill (paint/fill 0xFFe0e0e0)
        active-fill (paint/fill 0xFF78bc78)
        missed-active-fill (paint/fill 0xFFb1bc78)
        missed-inactive-fill (paint/fill 0xFFcce0cc)
        px-per-t 0.05]
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
                 (let [{:keys [line-sensors
                               line-switches
                               dt]} (nth history i)
                       seg-height (min (* px-per-t dt) rect-height)
                       y2 (unchecked-int (+ y seg-height))
                       draw-seg
                       (fn [n x]
                         (let [white? (= :white (nth line-sensors n))
                               nswitches (nth line-switches n)
                               mseg-height (/ seg-height (inc nswitches))]
                           (when white?
                             (.drawRect cnv
                               (Rect/makeXYWH x y seg-width seg-height)
                               active-fill))
                           (loop [i 1
                                  on? (not white?)]
                             (when (<= i nswitches)
                               (.drawRect cnv
                                 (Rect/makeXYWH x (+ y (* i mseg-height))
                                   seg-width mseg-height)
                                 (if on?
                                   missed-active-fill
                                   missed-inactive-fill))
                               (recur (inc i) (not on?))))))]
                     (draw-seg 0 0)
                     (draw-seg 1 seg-width)
                     (draw-seg 2 (* 2 seg-width))
                     (draw-seg 3 (* 3 seg-width))
                 (recur (unchecked-dec i) y2))))))}))))

(def ui-latency-graph-overlay
  (ui/halign 0
    (ui/valign 1
      (ui/padding 10 10
        (ui/dynamic ctx
          [client (:client ctx)]
          (let
            [ui-dropped
             (ui/clickable
               {:on-click
                (fn [_]
                  (swap! (client/get-req-status-atom client)
                    assoc
                    :requests-dropped 0
                    :requests-completed 0))}
               (ui/dynamic _
                 [{ndropped :requests-dropped
                   ncompleted :requests-completed}
                  @(client/get-req-status-atom client)]
                 (ui/label
                   (let [nreqs (+ ndropped ncompleted)]
                     (str ndropped #_#_"/" nreqs " ("
                       (format "%.2f"
                         (float
                           (if (zero? nreqs)
                             0.
                             (* 100 (/ ndropped nreqs)))))
                       "%) lost")))))
             ui-latency
             (ui/dynamic ctx
               [{:keys [dt]} (peek (:readings-history
                                     (:robot-state ctx)))]
               (ui/label (str dt " ms")))]
            (ui/column
              ui-latency
              (ui/gap 0 5)
              ui-dropped)))))))

(def ui-latency-graph
  (let [;bg-fill (paint/fill 0xFFf7f7f7)
        fill (paint/fill 0xFFced9dd)
        px-per-t 0.2
        seg-width 2]
    (ui/width 100
      (ui/with-bounds :latency-graph-bounds
        (ui/dynamic ctx
          [{:keys [latency-graph-bounds]} ctx
           history (:readings-history (:robot-state ctx))
           end (count history)
           start (max 0 (int (- end (Math/ceil (/ (:width latency-graph-bounds) seg-width)))))
           visible-history (subvec history start end)]
          (ui/stack
           (ui/canvas
             {:on-paint
              (fn [ctx ^Canvas cnv ^IPoint size]
                ;(.drawRect cnv (Rect/makeWH (.toPoint size)) bg-fill)
                (let
                  [scale (:scale ctx)
                   px-per-t (* scale px-per-t)
                   seg-width (* scale seg-width)
                   rect-height (:height size)]
                  (reduce
                    (fn [x {:keys [dt]}]
                      (let [seg-height (min (* px-per-t dt) rect-height)
                            x2 (+ x seg-width)]
                        (.drawRect cnv
                          (Rect/makeLTRB
                            x (- rect-height seg-height) x2 rect-height)
                          fill)
                        x2))
                    (max 0 (- (:width size) (* (- end start) seg-width)))
                    visible-history)))})
           (ui/dynamic ctx
             [{:keys [latency-graph-bounds]} ctx
              max-t (when (< 0 (count visible-history))
                      (reduce max (mapv :dt visible-history)))
              min-t (when (< 0 (count visible-history))
                      (reduce min (mapv :dt visible-history)))]
             (ui/label (str "(top " (int (/ (:height latency-graph-bounds) px-per-t))
                         " ms"
                         (when (< 0 (count visible-history))
                           (str ", " min-t "–" max-t " ms"))
                         ")")))
           (ui/dynamic _
             [ui-latency-graph-overlay ui-latency-graph-overlay]
             ui-latency-graph-overlay)))))))

(def *select-sim? (atom false))

(def ui-block-status
  (let [inactive-fill (paint/fill 0x4F000000)
        active-fill (paint/fill 0xFF40F040)
        green-fill (paint/fill 0xFFa6f7a0)
        red-fill (paint/fill 0xFFf7a0a0)]
    (ui/row
      (ui/dynamic ctx
        [{:keys [block-density
                 block-present?]} (:robot-readings ctx)]
        (ui/rect (case block-present?
                   true active-fill
                   false inactive-fill
                   (paint/fill 0xFFFFFFFF))
          (ui/padding 3
            (ui/center
              (ui/label (case block-density
                          :high "HI"
                          :low "LO"
                          "??"))))))
      (ui/gap 5 0)
      (ui/dynamic ctx
        [{:keys [signal-block-density]} (:robot-input ctx)]
        (ui/rect
          (case signal-block-density
            :high red-fill
            :low green-fill
            nil inactive-fill)
          (ui/padding 3
            (ui/center
              (ui/label (case signal-block-density
                          :high "HI"
                          :low "LO"
                          "  ")))))))))

(def ui-client-controls
  (ui/dynamic ctx
    [{:keys [client]} ctx
     {:keys [robot]} ctx]
    (ui/row
      (ui/dynamic ctx
        [{:keys [client-loop]
          looping? :client-looping?} ctx]
        (action-checkbox
          {:state looping?
           :on-toggle (fn [_ checked?]
                        (if checked?
                          (loopth/start-loop! client-loop)
                          (loopth/stop-loop! client-loop)))}
          (ui/label "Loop")))
      (ui/gap 5 0)
      (ui/dynamic ctx
        [{:keys [robot]} ctx
         robot-auto? (:auto? @(:*state robot) false)]
        (action-checkbox
          {:state robot-auto?
           :on-toggle
           (fn [_ checked?]
             (when-not checked?
               (swap! (:*input robot) assoc
                 :motor-1 0 :motor-2 0))
             (swap! (:*state robot) assoc :auto? checked?))}
          (ui/label "Auto")))
      (ui/gap 3 0)
      (ui/dynamic ctx [{:keys [robot]} ctx]
        (ui/button
          (fn []
            (swap! (:*state robot) assoc :auto? false)
            (swap! (:*input robot) assoc
              :motor-1 0 :motor-2 0))
          (ui/label "Stop")))
      (ui/gap 3 0)
      (ui/button
        (fn []
          (future
            (client/reset-connection! client
              (client/-get-connection client))))
        (ui/label "Reset"))
      (ui/gap 5 0)
      (ui/dynamic _ [conn-status (client/-get-status
                                   (client/-get-connection client))]
        (ui/valign 0.5 (ui/label (name conn-status))))
      (ui/gap 5 0)
      (ui/checkbox *select-sim? (ui/label "Sim"))
      (ui/gap 5 0)
      (ui/button
        (fn []
          (swap! (:*input robot) update
            :grabber-position
            #(if (= :open %) :closed :open)))
        (ui/label "Grabber")))))

(def ui-phase-select
  (let [*history (atom {:entries []
                        :idx 0})
        nav-history
        (fn [n]
          (let [{:keys [entries idx]}
                (swap! *history
                  (fn [{:keys [idx entries] :as h}]
                    (let [idx' (+ idx n)]
                      (if (<= 0 idx' (count entries))
                        (assoc h :idx idx')
                        h))))]
            (nth entries idx nil)))
        add-history
        (fn [text]
          (swap! *history
            (fn [{:keys [entries idx]}]
              (let [entries (conj (filterv (complement #{text})
                                    (subvec entries 0 idx))
                              text)]
                {:entries entries
                 :idx (count entries)}))))]
    (ui/dynamic ctx [{{*robot-state :*state} :robot} ctx]
      (ui/row
        (ui/dynamic ctx
          [phase-id (:phase-id @*robot-state)]
          (let [*state (atom {:placeholder "phase-id"
                              :text (some-> phase-id name)})
                reset-text! (fn [text]
                              (let [n (count text)]
                                (swap! *state assoc
                                  :text text
                                  :from n :to n)))
                submit!
                (fn [text]
                  (when (pos? (count text))
                    (let [phase-id (keyword text)]
                      (try
                        (swap! *robot-state
                          phase/init-phase-id-on-state phase-id)
                        (add-history text)
                        (catch Exception _
                          (println "Failed to set phase-id to" phase-id))))))]
            (add-watch *state :listener
              (fn [_ _ _ {:keys [text]}]
                ))
            (ui/stack
              (ui/text-field *state)
              (ui/text-listener
                {:on-input (fn [text] (= "\r" text))}
                (ui/key-listener
                 {:on-key-down
                  (fn [{:keys [key]}]
                    (if-some [text (case key
                                     :up (nav-history -1)
                                     :down (nav-history 1)
                                     nil)]
                      (do (reset-text! text)
                        true)
                      (case key
                        :enter (do (submit! (:text @*state))
                                 true)
                        false)))}
                 (ui/gap 100 0))))))
        (ui/button
          (fn [] (reset! *robot-state
                   (phase/init-phase-id-on-state
                     (merge robot.state/initial-state
                       (select-keys @*robot-state
                         [:readings-history :auto?
                          :next-phase-map]))
                     (:phase-id @*robot-state))))
          (ui/label "Clean"))))))

(def ui-raw-state-display
  (let [fmt-data #(zprint.core/zprint-str %
                    {:map {:justify? true
                           :justify {:max-variance 20}
                           :key-ignore [:history]}
                     :width 60})]
    (ui/column #_
      (ui/dynamic ctx
        [readings (:robot-readings ctx)]
        (multiline-label
          (fmt-data (dissoc readings
                      :line-sensors
                      :ultrasonic-1
                      :ultrasonic-2
                      :block-density
                      :block-present?
                      :dt
                      :time-received))))
      (ui/gap 0 8)
      (ui/dynamic ctx
        [state (:robot-state ctx)]
        (multiline-label
          (fmt-data (dissoc state :readings-history
                      :next-phase-map))))
      (ui/gap 0 8)
      (ui/dynamic ctx
        [input (:robot-input ctx)]
        (multiline-label
          (fmt-data (dissoc input :motor-1 :motor-2)))))))

(def ui-mainpage
  (ui/dynamic _ [ui-line-sensors ui-line-sensors
                 ui-motors ui-motors
                 ui-ultrasonics ui-ultrasonics
                 ui-ultrasonics-graph ui-ultrasonics-graph
                 ui-line-sensors-graph ui-line-sensors-graph
                 ui-client-controls ui-client-controls
                 ui-latency-graph ui-latency-graph
                 ui-block-status ui-block-status
                 ui-phase-select ui-phase-select
                 ui-raw-state-display ui-raw-state-display]
    (ui/row
      ui-line-sensors-graph
      [:stretch 1
       (ui/column
         (ui/padding 5
           (ui/row
             ui-line-sensors
             (ui/gap 8 0)
             ui-block-status
             (ui/gap 10 0)
             ui-client-controls))
         (ui/padding 5 0 ui-motors)
         (ui/gap 0 5)
         (ui/padding 5 0 ui-ultrasonics)
         [:stretch 1
          (ui/column ;; column to ensure scroll gets measured and updated
            (ui/vscrollbar
              (ui/vscroll
                (ui/padding 5 0 5 5
                  ui-raw-state-display))))]
         (ui/row
           ui-phase-select
           (ui/gap 5 0)
           (ui/dynamic ctx [{:keys [robot]} ctx]
             (ui/button (fn [] (robot.state/reset-robot! robot))
               (ui/label "Clear state")))
           (ui/gap 5 0)
           (ui/dynamic ctx [*robot-state (:*state (:robot ctx))]
             (ui/center
               (ui/clickable
                 {:on-click
                  (fn [_]
                    (swap! *robot-state update :readings-history empty))}
                 (ui/dynamic ctx
                   [npoints (count (:readings-history (:robot-state ctx)))]
                   (ui/label (str "Responses: " npoints)))))))
         (ui/gap 0 5)
         (ui/height 100 ui-latency-graph))]
      ui-ultrasonics-graph)))

(def ui-root
  (ui/mouse-listener
    {:on-move (fn [_] true)}
    (ui/stack
      (ui/canvas
        ;; Keep the frames coming
        {:on-event (fn [_ e] (= :frame (:event e)))})
      (ui/dynamic ctx
        [{:keys [scale]} ctx
         client-loop (if @*select-sim?
                       autopilot/*sim-loop
                       autopilot/*net-loop)
         net-robot robot.state/net-robot
         sim-robot robot.state/sim-robot
         robot (if @*select-sim? sim-robot net-robot)
         readings @(:*readings robot)
         robot-state @(:*state robot)
         robot-input @(:*input robot)
         client @(if @*select-sim?
                   sim.client/*client
                   net.api/*client)
         looping? (loopth/loop-running? client-loop)
         ui-mainpage ui-mainpage]
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
          ui-mainpage)))))

(def *app (atom nil))
(reset! *app
  (ui/dynamic _ [ui-root ui-root
                 wc common/with-context]
    (wc ui-root)))

(def *window (agent nil))

(defn open-window! []
  (send *window
    (fn [^Window prev-window]
      (app/doui
        (if (or (nil? prev-window)
              (window/closed? prev-window))
          (ui/window
            {:title "IDP Client Monitor"
             :bg-color 0xFFFFFFFF
             :width 1000
             :height 800
             :exit-on-close? false}
            *app)
          (do (.focus prev-window)
            prev-window))))))
