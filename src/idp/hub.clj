(ns idp.hub
  "The main program window UI.
  Closing this window exits the program.
  Used to access other windows (monitor, simulation)"
  (:require
    [io.github.humbleui.app :as app]
    [io.github.humbleui.ui :as ui]
    [idp.robot.sim.ui :as sim.ui]
    [idp.robot.monitor.panel :as monitor.panel]
    [idp.robot.brain.phase :as phase]
    [idp.common :as common]
    [idp.robot.state :as robot.state]
    [idp.loopthread :as loopth]
    [idp.robot.autopilot :as autopilot])
  (:import
    (io.github.humbleui.jwm Window)))

(def *sim? (atom false))

(def ui-root
  (ui/dynamic _ [with-context common/with-context
                 [client-loop
                  robot] (if @*sim?
                           [autopilot/*sim-loop
                            robot.state/sim-robot]
                           [autopilot/*net-loop
                            robot.state/net-robot])]
    (with-context
      (ui/mouse-listener
        {:on-move (fn [_] true)}
        (ui/column
          (ui/button (fn [] (monitor.panel/open-window!))
            (ui/label "Monitor"))
          (ui/gap 0 5)
          (ui/button
            (fn [] (sim.ui/open-sim-window!))
           (ui/label "Simulation"))
          (ui/gap 0 10)
          (ui/column
            (ui/checkbox *sim?
              (ui/label "Sim?"))
            (ui/button
              ;; This button begins the full competition attempt
              (fn []
                (loopth/start-loop! client-loop)
                (swap! (:*state robot) assoc
                  :auto? true
                  :phase (phase/get-initial-state
                           (phase/lookup-phase :full-run))))
              (ui/label "Start"))
            (ui/gap 0 3)
            (ui/button
              ;; Stops the robot's motion and disables autopilot
              ;; but remains connected
              (fn []
                (swap! (:*state robot) assoc :auto? false)
                (swap! (:*input robot) assoc
                  :motor-1 0 :motor-2 0))
              (ui/label "Stop"))
            (ui/gap 0 5)
            (ui/button
              ;; Cuts off the connection with the Arduino
              (fn []
                (loopth/stop-loop! client-loop)
                (swap! (:*state robot) assoc :auto? false)
                (swap! (:*input robot) assoc
                  :motor-1 0 :motor-2 0))
              (ui/label "Disconnect"))))))))

(def *app (atom nil))
(reset! *app
  (ui/dynamic _ [ui-root ui-root] ui-root))

(def *window (promise))

(defn open-hub-window!
  "Opens the main window with a simple list of buttons.
  Closing this window exits the program."
  []
  (app/doui-async
    (deliver *window
      (doto ^Window
        (ui/window
          {:title "IDP Hub"
           :bg-color 0xFFFFFFFF
           :width 260
           :height 350
           :exit-on-close? true}
          *app)
        .focus))))
