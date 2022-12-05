(ns idp.robot.monitor.panel
  "Live visualisations of the robot sensor data and connection status."
  (:require
    [puget.printer :as puget]
    [idp.robot.state :as robot.state]
    [idp.robot.brain.phase :as phase]
    [io.github.humbleui.app :as app]
    [io.github.humbleui.paint :as paint]
    [idp.common :as common :refer [multiline-label
                                   action-checkbox]]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window]
    [io.github.humbleui.font :as font]
    [idp.robot.autopilot :as autopilot]
    [idp.net.api :as net.api]
    [idp.robot.sim.client :as sim.client]
    [idp.robot.client :as client]
    [idp.loopthread :as loopth])
  (:import
    (io.github.humbleui.jwm Window)
    (io.github.humbleui.types IPoint Rect)
    (io.github.humbleui.skija Canvas)))

(def ui-line-sensor-dot
  "Coloured circle indicating a line sensor readings.
  Line present → green
  No line → grey"
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
  "A row of circles indicating the latest line sensor readings"
  (ui/dynamic ctx
    [ui-line-sensor-dot ui-line-sensor-dot
     line-sensors (:line-sensors
                    (:robot-readings ctx))
     ui-ls (fn [ls]
             (ui/with-context
               {:line-sensor-reading ls}
               ui-line-sensor-dot))]
    (let [gap 5]
      (ui/row
        (eduction
          (map ui-ls)
          (interpose (ui/gap gap 0))
          line-sensors)))))

(def ui-rect-meter
  "A rectangular bar displaying a fractional quantity (:meter-value)
  which may have the range [0,1] or [-1,1]."
  (let [border-stroke (paint/stroke 0xFF505050 3)
        fallback-fill (paint/fill 0xFFb3efef)
        fallback-nodata-fill (paint/fill 0xFFc7dbdb)]
    (ui/dynamic _ []
      (ui/rect border-stroke
        (ui/padding 1
          (ui/canvas
            {:on-paint
             (fn [ctx ^Canvas cnv ^IPoint size]
               (if-some [coeff (:meter-value ctx)]
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
                     fill))
                 (let [no-data-fill (:no-data-fill ctx fallback-nodata-fill)]
                   (.drawRect cnv (Rect/makeWH (.toPoint size)) no-data-fill))))}))))))

(def ui-motor
  "Displays a horizontal bar showing a motor speed
  obtained from :motor-speed in the UI context"
  (let [forward-fill (paint/fill 0xFFc3ddf7)
        backward-fill (paint/fill 0xFFf7e7c0)]
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
  "Displays the requested motor speeds"
  (ui/height 40
    (ui/dynamic ctx [ui-motor ui-motor
                     motor-1 (:motor-1 (:robot-input ctx))
                     motor-2 (:motor-2 (:robot-input ctx))]
      (ui/column
        [:stretch 1 (ui/with-context {:motor-speed motor-1} ui-motor)]
        [:stretch 1 (ui/with-context {:motor-speed motor-2} ui-motor)]))))

(defn ui-ultrasonic
  "Makes a component that displays an ultrasonic reading
  as a horizontal bar."
  [us-key]
  (let [max-dist 800]
    (ui/dynamic _ [ui-rect-meter ui-rect-meter]
      (ui/stack
        (ui/dynamic ctx [dist (us-key (:robot-readings ctx))]
          (let [coeff (float (/ dist max-dist))]
            (ui/with-context
              {:meter-value (when (pos? dist) coeff)}
              ui-rect-meter)))
        (ui/dynamic ctx [dist (us-key (:robot-readings ctx))]
          (ui/valign 0.5
            (ui/padding 5 0 0 0
              (ui/label (if (zero? dist)
                          "—"
                          (str (format "%.1f" (float dist)) " mm"))))))))))

(def ui-ultrasonics
  (ui/dynamic _ [ui-ultrasonic ui-ultrasonic]
    (ui/height 40
      (ui/column
        [:stretch 1 (ui-ultrasonic :ultrasonic-1)]
        [:stretch 1 (ui-ultrasonic :ultrasonic-2)]))))

(def ui-ultrasonics-graph
  "Displays a graph of ultrasonic readings over time.
  There are two columns, one for each sensor (left, rear).
  Readings are displayed as if zero were at the centre.
  A solid grey colour is shown when there are no readings available.
  The most recent readings are at the top."
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
               (fn [y {:keys [ultrasonic-1 ultrasonic-2] :as reading}]
                 (let [seg-height (min (* px-per-t
                                         (robot.state/get-active-dt reading))
                                    rect-height)
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
  "Displays the line sensor readings over time.
  Consists of four columns, one for each line sensor.
  If a line is detected, green is displayed.
  The most recent readings are at the top."
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
                               line-switches] :as reading} (nth history i)
                       seg-height (min (* px-per-t (robot.state/get-active-dt reading)) rect-height)
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
  "Shows additional information regarding the Wi-Fi connection
  such as the amount of responses that failed to come through in time
  and the time since the previous response."
  (ui/halign 0
    (ui/valign 1
      (ui/padding 10 10
        (ui/dynamic ctx
          [client (:client ctx)]
          (let
            [ui-dropped
             (ui/clickable
               {:on-click
                (fn [_] ;; Click to clear the history of responses
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
                     (str ndropped " ("
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
               (ui/label (if dt
                           (str (format "%.1f" (double dt)) " ms")
                           "-")))]
            (ui/column
              ui-latency
              (ui/gap 0 5)
              ui-dropped)))))))

(def ui-latency-graph
  "A graph that plots the time between each response (Δt).
  The most recent response is shown on the right."
  (let [fill (paint/fill 0xFFced9dd)
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
                           (str ", " (format "%.1f" (double min-t))
                             "–" (format "%.1f" (double max-t)) " ms"))
                         ")")))
           (ui/dynamic _
             [ui-latency-graph-overlay ui-latency-graph-overlay]
             ui-latency-graph-overlay)))))))

(def *select-sim?
  "Whether to view data from the simulation or the real robot"
  (atom false))

(def ui-client-controls
  (ui/dynamic ctx
    [{:keys [client]} ctx]
    (ui/row
      (ui/dynamic ctx
        [{:keys [client-loop]
          looping? :client-looping?} ctx]
        ;; Determines whether the autopilot loop is running
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
        ;; Determines whether the autopilot will be updated with
        ;; each loop cycle
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
        ;; Emergency button that stops the robot's motion and
        ;; disables the autopilot
        (ui/button
          (fn []
            (swap! (:*state robot) assoc :auto? false)
            (swap! (:*input robot) assoc
              :motor-1 0 :motor-2 0))
          (ui/label "Stop")))
      (ui/gap 3 0)
      (ui/button
        ;; Forces a reconnection
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
      (ui/checkbox *select-sim? (ui/label "Sim")))))

(def *phase-select-history (atom {:entries [] :idx 0}))

(defn phase-select-add-history! [text]
  (swap! *phase-select-history
    (fn [{:keys [entries idx]}]
      (let [entries (conj (filterv (complement #{text})
                            (subvec entries 0 idx))
                      text)]
        {:entries entries
         :idx (count entries)}))))

(defn phase-select-submit-command! [robot text]
  (when (pos? (count text))
    (try
      (let [[sym params overlap]
            (binding [*read-eval* false]
              (read-string (str "[" text "]")))
            phase-id (keyword sym)]
        (swap! (:*state robot) assoc :phase
          (phase/merge-states
            (phase/get-initial-state (phase/lookup-phase phase-id) params)
            overlap))
        (phase-select-add-history! text))
      (catch Exception e
        (println "Failed to set phase-id" e)))))

(defn phase-select-nav-history!
  "Traverses the history of phase commands by a delta of n.
  If n is negative, a previous command is selected."
  [n]
  (let [{:keys [entries idx]}
        (swap! *phase-select-history
          (fn [{:keys [idx entries] :as h}]
            (let [idx' (+ idx n)]
              (if (<= 0 idx' (dec (count entries)))
                (assoc h :idx idx')
                h))))]
    (nth entries idx nil)))

(def ui-phase-select
  "A textbox that allows initialising a root autopilot phase.
  The content is interpreted as though it were within a vector.
  The first item is a symbol representing the phase-id (non-optional).
  The second item is the 'params' map used to initialise the phase state.
  The third item is a map that gats merged into the phase state.
  Using the arrow keys, you can navigate between previously submitted commands."
  (let [*state (atom {:placeholder "phase-id"
                      :text (peek (:entries @*phase-select-history))})]
    (ui/dynamic ctx [{:keys [robot]} ctx]
      (ui/row
        (ui/dynamic _ []
          (let [reset-text! (fn [text]
                              (let [n (count text)]
                                (swap! *state assoc
                                  :text text
                                  :from n :to n)))
                submit!
                (fn [text] (phase-select-submit-command! robot text))]
            (ui/stack
              (ui/text-field *state)
              (ui/text-listener
                {:on-input (fn [text] (= "\r" text))}
                (ui/key-listener
                 {:on-key-down
                  (fn [{:keys [key]}]
                    (if-some [text (case key
                                     :up (phase-select-nav-history! -1)
                                     :down (phase-select-nav-history! 1)
                                     nil)]
                      (do (reset-text! text)
                        true)
                      (case key
                        :enter (do (submit! (:text @*state))
                                 true)
                        false)))}
                 (ui/gap 200 0))))))
        (ui/button
          (fn [] (phase-select-submit-command! robot (:text @*state)))
          (ui/label "Go"))))))

(def ui-raw-state-display
  "Displays the time elapsed and a textual representation of
  the root phase state"
  (let [fmt-data #(puget/pprint-str %
                    {:width 60
                     :coll-limit 7
                     :map-coll-separator :line})]
    (ui/column
      (ui/dynamic ctx
        [start-time (:competition-start-time
                      (:phase (:robot-state ctx)))
         t (System/currentTimeMillis)]
        (ui/label
          (if start-time
            (str "Time elapsed: "
              (min 9999 (long (/ (- t start-time) 1000)))
              "/300")
            "-")))
      (ui/dynamic ctx
        [state (:robot-state ctx)]
        (multiline-label
          (fmt-data (:phase state)))))))

(def ui-block-status
  "Shows block presence and density readings,
  and the desired block density LED state."
  (let [inactive-fill (paint/fill 0x4F000000)
        active-fill (paint/fill 0xFF40F040)
        green-fill (paint/fill 0xFFa6f7a0)
        red-fill (paint/fill 0xFFf7a0a0)]
    (ui/row
      (ui/dynamic ctx
        [{:keys [block-density
                 block-present?]} (:robot-readings ctx)]
        ;; Green background if block present
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
      (ui/valign 0.5 (ui/label "Signal:"))
      (ui/dynamic ctx
        [{:keys [signal-block-density]} (:robot-input ctx)
         {:keys [robot]} ctx]
        (ui/clickable
          {:on-click ;; Click to manually set the block density LED
           (fn [_] (swap! (:*input robot) update :signal-block-density
                     {:high :low
                      :low nil
                      nil :high}))}
          (ui/rect
            (case signal-block-density
              :high red-fill
              :low green-fill
              nil inactive-fill)
            (ui/padding 4
              (ui/center
                (ui/label (case signal-block-density
                            :high "HI"
                            :low "LO"
                            "  "))))))))))

(def ui-top-indicators
  "Shows the desired grabber position and
  whether ultrasonic sensors are enabled.
  Buttons allow setting these."
  (ui/dynamic _ [ui-block-status ui-block-status
                 ui-line-sensors ui-line-sensors]
    (ui/row
      ui-line-sensors
      (ui/gap 8 0)
      ui-block-status
      (ui/gap 5 0)
      (ui/dynamic ctx
        [{{:keys [grabber-position]} :robot-input
          :keys [robot]} ctx]
        (ui/valign 0.5
          (ui/with-context
            {:hui.button/bg (paint/fill 0)}
            (ui/button
              (fn []
                (swap! (:*input robot) update
                  :grabber-position
                  #(if (= :open %) :closed :open)))
              (ui/label (str "Grabber: "
                          (case grabber-position
                            :open "open"
                            :closed "closed"
                            "?")))))))
      (ui/gap 5 0)
      (ui/dynamic ctx
        [{{:keys [ultrasonic-active?]} :robot-input
          :keys [robot]} ctx]
        (ui/valign 0.5
          (ui/button
            (fn []
              (swap! (:*input robot) update
                :ultrasonic-active? not))
            (ui/label (str "Ultrasonic: "
                        ultrasonic-active?))))))))

(def ui-mainpage
  (ui/dynamic _ [ui-motors ui-motors
                 ui-ultrasonics ui-ultrasonics
                 ui-ultrasonics-graph ui-ultrasonics-graph
                 ui-line-sensors-graph ui-line-sensors-graph
                 ui-client-controls ui-client-controls
                 ui-latency-graph ui-latency-graph
                 ui-top-indicators ui-top-indicators
                 ui-phase-select ui-phase-select
                 ui-raw-state-display ui-raw-state-display]
    (ui/row
      ui-line-sensors-graph
      [:stretch 1
       (ui/column
         (ui/padding 4 3 ui-top-indicators)
         (ui/padding 4 3 ui-client-controls)
         (ui/padding 4 0 ui-motors)
         (ui/gap 0 5)
         (ui/padding 4 0 ui-ultrasonics)
         [:stretch 1
          (ui/column ;; column within :stretch to ensure scroll gets measured and updated
            (ui/vscrollbar
              (ui/vscroll
                (ui/padding 5
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
                   (ui/label (str npoints " responses")))))))
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

(defn open-window!
  "Opens and focuses the monitor window"
  []
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
