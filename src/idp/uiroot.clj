(ns idp.uiroot
  (:require
    [io.github.humbleui.app :as app]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.debug :as debug]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window]
    [idp.state :as state]
    [idp.common :as common]
    [idp.board.background :as board.bg]
    [idp.board.params :as board.params]
    [idp.board.geo :as board.geo]
    [idp.robot.graphic :as robot.g]
    [idp.robot.sim.ui :as robot.sim.ui]
    [idp.robot.state :as robot.state])
  (:import
    [io.github.humbleui.skija ColorSpace]
    [io.github.humbleui.jwm Window]
    [io.github.humbleui.jwm.skija LayerMetalSkija]))

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
                  {:on-paint
                   (fn [ctx _ _]
                     (hui/schedule
                       #(window/request-frame (:window ctx))
                       0))})))))))))

(def ui-root
  (ui/dynamic ctx
    [ui-arena ui-arena
     sim-controls robot.sim.ui/ui-controls]
    (ui/column
      (ui/halign 0.5
        (ui/height
          (fn [{:keys [width height]}]
            (min height width))
          (ui/width #(:height %)
            ui-arena)))
      sim-controls)))

(def app
  (ui/dynamic ctx [ui-root ui-root
                   wc common/with-context]
    (wc ui-root)))

(reset! state/*app app)

(defn open-sim-window! []
  (app/doui-async
    (deliver state/*window
      (ui/window
        {:title    "IDP"
         :bg-color 0xFFFFFFFF
         :exit-on-close? false}
        state/*app))))

(defn start-ui! [& _args]
  (ui/start-app!)
  ; (reset! debug/*enabled? true)
  #_(common/redraw))

(comment
  (open-sim-window!)
  )