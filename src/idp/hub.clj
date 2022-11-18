(ns idp.hub
  (:require
    [io.github.humbleui.app :as app]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.debug :as debug]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window]
    [idp.state :as state]
    [idp.robot.sim.ui :as sim.ui]
    [idp.robot.monitor.panel :as monitor.panel]
    [idp.common :as common])
  (:import
    (io.github.humbleui.jwm Window)))

(def ui-root
  (ui/dynamic _ [with-context common/with-context]
    (with-context
      (ui/mouse-listener
        {:on-move (fn [_] true)}
        (ui/column
          (ui/button (fn [] (monitor.panel/open-window!))
            (ui/label "Monitor"))
          (ui/gap 0 5)
          (ui/button
            (fn []
              (sim.ui/open-sim-window!))
           (ui/label "Simulation")))))))

(def *app (atom nil))
(reset! *app
  (ui/dynamic _ [ui-root ui-root] ui-root))

(def *window (promise))

(defn open-hub-window! []
  (app/doui-async
    (deliver *window
      (doto ^Window
        (ui/window
          {:title "IDP Hub"
           :bg-color 0xFFFFFFFF
           :width 200
           :height 100
           :exit-on-close? true}
          *app)
        .focus))))

(app/doui-async
  (window/request-frame @*window))