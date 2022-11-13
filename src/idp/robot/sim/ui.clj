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


(def ui-angle-slider
  (ui/padding 15 3
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
          (ui/dynamic ctx
            [typeface common/mono-typeface
             scale (:scale ctx)]
            (ui/with-context
              {:font-ui (font/make-with-cap-height typeface (* 10 scale))}
              (ui/dynamic _ [angle (:angle @robot.state/*real)]
                (ui/label (format "%#5.1f°" (unchecked-double angle)))))))))))

(def ui-controls
  (ui/dynamic _ [ui-angle-slider ui-angle-slider
                 multiline-label common/multiline-label]
    (ui/column
      ui-angle-slider
      (ui/gap 0 2)
      (ui/center
        (ui/row
          (ui/button
            (fn [] (robot.state/reset-real!))
            (ui/label "Move to start"))
          (ui/gap 10 0)
          (ui/dynamic _ [looping? (loopth/loop-running? sim/*loop)]
            (let [*looping (atom looping?)]
              (add-watch *looping :checkbox
                (fn [_ _ _looping? looping?']
                  (if looping?'
                    (loopth/start-loop! sim/*loop)
                    (loopth/stop-loop! sim/*loop))))
              (ui/checkbox
                *looping
                (ui/label "Sim loop"))))))
      (ui/dynamic _ [state @(:*state @sim.client/*client)]
        (multiline-label
         (zprint/zprint-str
           (dissoc state
             :ready-req-queue :ready-res-queue :res-queue :req-queue)))))))
