(ns idp.robot.sim.ui
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
    [idp.robot.sim.client :as sim.client]))

(defn with-monospace [child]
  (ui/dynamic ctx
    [typeface common/mono-typeface
     scale (:scale ctx)]
    (ui/with-context
      {:font-ui (font/make-with-cap-height typeface (* 10 scale))}
      child)))

(def ui-angle-slider
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

(def ui-controls
  (ui/dynamic _ [ui-angle-slider ui-angle-slider
                 multiline-label common/multiline-label]
    (ui/padding 6 0
      (ui/column
        ui-angle-slider
        (ui/gap 0 2)
        (ui/center
          (ui/row
            (ui/button
              (fn [] (robot.state/reset-real!))
              (ui/label "Move to start"))
            (ui/gap 10 0)
            (ui/dynamic _ [looping? (sim/sim-running?)]
              (let [*looping (atom looping?)]
                (add-watch *looping :checkbox
                  (fn [_ _ _ looping?]
                    (if looping?
                      (sim/start!)
                      (sim/stop!))))
                (ui/checkbox *looping
                  (ui/label "Sim loop"))))))
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
