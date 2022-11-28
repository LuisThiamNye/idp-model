(ns idp.hub
  "The main program window UI.
  Closing this window exits the program.
  Used to access other windows (monitor, simulation)"
  (:require
    [io.github.humbleui.app :as app]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window]
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
            (fn []
              (sim.ui/open-sim-window!))
           (ui/label "Simulation"))
          (ui/gap 0 10)
          (ui/column
            (ui/checkbox *sim?
              (ui/label "Sim?"))
            (ui/button
              (fn []
                (loopth/start-loop! client-loop)
                
                (swap! (:*state robot)
                  (fn [state]
                    (-> state
                      (assoc :auto? true)
                      (phase/initialise-phase-on-state
                        (phase/lookup-phase :exit-start)))))
                (swap! (:*state robot) assoc :next-phase-map
                  {:exit-start :exit-start-turn
                   ; :exit-start-turn :start-to-centre-block
                   :exit-start-turn :start-to-centre-block-tunnel
                   [:start-to-centre-block-tunnel] :detect-block
                   :start-to-centre-block :detect-block
                   :detect-block :centre-block-180
                   :centre-block-180 :tunnel-approach
                   :tunnel-approach :through-tunnel
                   :through-tunnel :up-to-box
                   :up-to-box :box-approach-turn
                   :box-approach-turn :box-approach-turn-spin
                   :box-approach-turn-spin :box-approach-edge
                   :box-approach-edge :backup-from-box
     
                   [:backup-from-box {:go-home? true}] :up-to-home-entry
                   :up-to-home-entry :align-to-home
     
                   [:backup-from-box {:go-home? false}] :start-to-centre-block
                   }
                  )
                )
              (ui/label "Start"))
            (ui/gap 0 3)
            (ui/button
              (fn []
                (swap! (:*state robot) assoc :auto? false)
                (swap! (:*input robot) assoc
                  :motor-1 0 :motor-2 0))
              (ui/label "Stop"))
            #_#_(ui/gap 0 5)
            (ui/button
              (fn []
                (swap! (:*state robot)
                  (fn [state]
                    (-> state
                      (assoc
                        :auto? true
                        :next-phase-map
                        {:detect-block :stationary-open-grabber
                         :stationary-open-grabber :stop})
                      (phase/init-phase-id-on-state :detect-block)))))
              (ui/label "Detect"))
            (ui/gap 0 5)
            (ui/button
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

(defn open-hub-window! []
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
