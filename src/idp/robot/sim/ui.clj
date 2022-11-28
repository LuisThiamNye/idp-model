(ns idp.robot.sim.ui
  "Robot simulation window UI"
  (:require
    [clojure.string :as str]
    [zprint.core :as zprint]
    [idp.robot.params :as params]
    [idp.robot.state :as robot.state]
    [idp.robot.sim.client :as sim.client]
    [idp.board.geo :as board.geo]
    [idp.state :as state]
    [io.github.humbleui.app :as app]
    [io.github.humbleui.paint :as paint]
    [idp.common :as common]
    [idp.sim :as sim]
    [idp.loopthread :as loopth]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.canvas :as canvas]
    [io.github.humbleui.window :as window]
    [io.github.humbleui.font :as font]
    [io.github.humbleui.font :as typeface]
    [idp.board.background :as board.bg]
    [idp.robot.graphic :as robot.g]
    [idp.board.params :as board.params]
    [idp.robot.sim.ui :as robot.sim.ui])
  (:import
    (io.github.humbleui.jwm Window)))

(defn with-monospace [child]
  (ui/dynamic ctx
    [typeface common/mono-typeface
     scale (:scale ctx)]
    (ui/with-context
      {:font-ui (font/make-with-cap-height typeface (* 10 scale))}
      child)))

(def ui-angle-slider
  "Slider that reflects and controls the robot's direction"
  (ui/padding 10 3
    (let [*state (atom nil)]
      (add-watch *state :slider
        (fn [_ _ prev {:keys [value]}]
          (when prev
            (let [angle' (* (/ value 100) 360)]
              (when (< 0.1 (abs (- angle' (:angle @robot.state/*real))))
                (swap! robot.state/*real assoc
                  :angle (max 0 (min 359.9999 (double angle')))))))))
      (add-watch robot.state/*real ::angle-slider
        (fn [_ _ _ {:keys [angle]}]
          (let [value' (min 100 (max 0 (/ angle 3.6)))]
            (when (< 0.1 (abs (- (:value @*state) value')))
              (swap! *state assoc :value (hui/clamp value' 0 100))))))
      (ui/row
        [:stretch 1 (ui/slider *state)]
        (ui/valign 0.5
          (with-monospace
            (ui/dynamic _ [angle (:angle @robot.state/*real)]
              (ui/label (format "%#5.1f°" (unchecked-double angle))))))))))

(def *savestate (atom {}))

(def ui-savestate-controls
  (ui/dynamic _ []
    (ui/row
      (ui/button
        (fn []
          (reset! *savestate (select-keys @robot.state/*real
                               [:position :angle])))
        (ui/label "Save"))
      (ui/gap 5 0)
      (ui/button
        (fn []
          (swap! robot.state/*real merge @*savestate))
        (ui/label "Load")))))

(def ui-controls
  (ui/dynamic _ [ui-angle-slider ui-angle-slider
                 ui-savestate-controls ui-savestate-controls
                 multiline-label common/multiline-label]
    (ui/padding 6 0
      (ui/column
        ui-angle-slider
        (ui/gap 0 2)
        (ui/center
          (ui/row
            (ui/gap 5 0)
            (ui/button
              (fn [] (robot.state/reset-real!))
              (ui/label "Move to start"))
            (ui/gap 5 0)
            (ui/checkbox robot.state/*sim-block-present?
              (ui/label "Block present?"))
            (ui/gap 5 0)
            (ui/button
              (fn [] (swap! robot.state/*sim-block-density
                       (fn [d] (case d :high :low :low :high))))
              (ui/dynamic _ [density @robot.state/*sim-block-density]
                (ui/label (str "Density: " (name density)))))))
        (ui/gap 0 5)
        (ui/center
          (ui/row
            ui-savestate-controls
            (ui/gap 5 0)
            (ui/dynamic _ [looping? (sim/sim-running?)]
              (common/action-checkbox
                {:state looping?
                 :on-toggle (fn [_ checked?]
                              (if checked?
                                (sim/start!)
                                (sim/stop!)))}
                (ui/label "Sim loop")))))
        (ui/row
          (ui/valign 0.5 (ui/label "Speed"))
          (ui/gap 5 0)
          [:stretch 1
           (let [*state (atom {:value @robot.state/*sim-speed
                               :step 0.2
                               :min 0
                               :max 10})]
             (add-watch *state :action
               (fn [_ _ _ {:keys [value]}]
                 (reset! robot.state/*sim-speed value)
                 (when (sim/sim-running?)
                   (sim/start!))))
             (ui/slider *state))]
          (with-monospace
            (ui/dynamic _ [sim-speed @robot.state/*sim-speed]
              (ui/label (format "%#4.1f" (float sim-speed))))))
        (ui/row
          (ui/valign 0.5 (ui/label "dt"))
          (ui/gap 5 0)
          [:stretch 1
           (let [*state (atom {:value (hui/clamp @robot.state/*sim-dt
                                        100 10000)
                               :step 10
                               :min 100
                               :max 10000})]
             (add-watch *state :action
               (fn [_ _ _ {:keys [value]}]
                 (reset! robot.state/*sim-dt value)
                 (when (sim/sim-running?)
                   (sim/start!))))
             (ui/slider *state))]
          (with-monospace
            (ui/dynamic _ [sim-dt @robot.state/*sim-dt]
              (ui/label (format "%#5.0f μs" (float sim-dt))))))
        (ui/dynamic _ [state @(:*state @sim.client/*client)]
          (multiline-label
            (zprint/zprint-str
              (dissoc state
                :ready-req-queue :ready-res-queue :res-queue :req-queue))))))))

(def ui-arena
  (ui/mouse-listener
    {:on-move (fn [_evt] false)}
    (ui/with-bounds :arena-bounds
      (ui/dynamic ctx
        [ui-background board.bg/ui-background
         ui-robot robot.g/ui-robot
         dims board.params/dims
         {:keys [board-width board-height]} dims
         size (:arena-bounds ctx)]
        (ui/with-context
          {:dims dims
           :board-scale (let [sf-x (/ (:x size) board-width)
                              sf-y (/ (:y size) board-height)]
                          (min sf-x sf-y))
           :*robot-real-state robot.state/*real}
          (ui/dynamic ctx [{:keys [board-scale scale]} ctx]
            (ui/mouse-listener
              {:on-button (fn [evt]
                            (when (and (= :primary (:button evt))
                                    (:pressed? evt))
                              (let [scale (* board-scale scale)
                                    pos {:x (/ (:x evt) scale) :y (/ (:y evt) scale)}]
                                (swap! robot.state/*real assoc :position pos))))
               :on-move (fn [evt]
                          (swap! state/*misc assoc :mouse-on-line?
                            (board.geo/point-on-line?
                              (let [scale (* board-scale scale)
                                    pos {:x (/ (:x evt) scale) :y (/ (:y evt) scale)}]
                                ; (prn pos)
                                pos))))}
              (ui/stack
                ui-background
                ui-robot
                (ui/canvas
                  {:on-event
                   (fn [_ e]
                     ;; Keep the frames coming
                     (= :frame (:event e)))})))))))))

(def ui-root
  (ui/dynamic ctx
    [ui-arena ui-arena
     sim-controls robot.sim.ui/ui-controls
     with-context common/with-context]
    (with-context
      (ui/column
        (ui/halign 0.5
          (ui/height
            (fn [{:keys [width height]}]
              (min height width))
            (ui/width #(:height %)
              ui-arena)))
        sim-controls))))

(def *app (atom nil))
(reset! *app
  (ui/dynamic _ [ui-root ui-root] ui-root))

(def *window (agent nil))

(defn open-sim-window! []
  (send *window
    (fn [^Window prev-window]
      (app/doui
        (if (or (nil? prev-window)
              (window/closed? prev-window))
          (ui/window
            {:title "IDP Simulation"
             :bg-color 0xFFFFFFFF
             :width 500
             :height 800
             :exit-on-close? false}
            *app)
          (do (.focus prev-window)
            prev-window))))))