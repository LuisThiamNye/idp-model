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
    [idp.robot.state :as robot.state])
  (:import
    [io.github.humbleui.skija ColorSpace]
    [io.github.humbleui.jwm Window]
    [io.github.humbleui.jwm.skija LayerMetalSkija]))

(def *last-tick-time (atom 0))
(def *sim-dt (atom 20))
(def *sim-speed (atom 2))

(defn sim-tick! [dt]
  ; (idp.robot.brain.travel/tick!)
  (robot.state/tick! dt))

(def *stop-sim? (atom false))

(defn sim-tick-recurring []
  (if @*stop-sim?
    (reset! *stop-sim? false)
    (let [dt @*sim-dt
          now (hui/now)
          del (max 0 (- (+ (/ dt @*sim-speed)
                          @*last-tick-time) now))]
      ; (prn del)
      (reset! *last-tick-time now)
      (sim-tick! dt)
      (hui/schedule #(sim-tick-recurring)
        del))))

(sim-tick-recurring)
(comment
  (reset! *stop-sim? true)
  )

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

(def app
  (common/with-context
    (ui/dynamic ctx [ui-arena ui-arena]
      ui-arena)))

(reset! state/*app app)

(defn start-ui! [& _args]
  (ui/start-app!
    (deliver state/*window
      (ui/window
        {:title    "IDP"
         :bg-color 0xFFFFFFFF}
        state/*app)))
  ; (reset! debug/*enabled? true)
  #_(common/redraw))

(comment
  (app/doui-async
    (start-ui!))
  )