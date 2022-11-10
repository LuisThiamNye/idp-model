(ns idp.robot.monitor.panel
  (:require
    [idp.robot.params :as params]
    [idp.robot.state :as robot.state]
    [idp.board.geo :as board.geo]
    [idp.state :as state]
    [io.github.humbleui.app :as app]
    [io.github.humbleui.paint :as paint]
    [idp.common :as common]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.core :as hui]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.canvas :as canvas]
    [io.github.humbleui.window :as window])
  (:import
    [io.github.humbleui.types IRect IPoint Rect]
    [io.github.humbleui.skija Canvas ImageFilter SaveLayerRec Paint]
    [java.lang AutoCloseable]))

(def ui-line-sensor-dot
  (let [inactive-fill (paint/fill 0x4F000000)
        active-fill (paint/fill 0xFF40F040)
        ]
    (ui/width 20
      (ui/height 20
        (ui/canvas
          {:on-paint
           (fn [ctx ^Canvas cnv size]
             (let [rx (/ (:width size) 2)
                   ry (/ (:height size) 2)]
               (.drawCircle cnv rx ry
                 (min rx ry)
                 (if (:line-sensor-triggered? ctx)
                   active-fill
                   inactive-fill))))})))))

(def ui-root
  (ui/mouse-listener
    {:on-move (fn [_] true)}
    (ui/stack
      (ui/canvas
        {:on-paint
         (fn [ctx _ _]
           (hui/schedule #(window/request-frame (:window ctx)) 20))})
      (ui/dynamic _
        [ui-line-sensor-dot ui-line-sensor-dot
         {:keys [line-sensor-1
                 line-sensor-2
                 line-sensor-3]} @robot.state/*real
         {:keys [mouse-on-line?]} @state/*misc]
       (ui/row
         (ui/with-context
           {:line-sensor-triggered? line-sensor-1}
           ui-line-sensor-dot)
         (ui/gap 10 0)
         (ui/with-context
           {:line-sensor-triggered? line-sensor-2}
           ui-line-sensor-dot)
         (ui/gap 10 0)
         (ui/with-context
           {:line-sensor-triggered? line-sensor-3}
           ui-line-sensor-dot)
         (ui/gap 20 0)
         (ui/with-context
           {:line-sensor-triggered? mouse-on-line?}
           ui-line-sensor-dot))))))

(def app
  (common/with-context
    (ui/dynamic ctx [ui-root ui-root]
      ui-root)))

(def *app (atom nil))
(reset! *app app)
(def *window (atom nil))

(defn open-window! []
  (reset! *window
    (ui/window
      {:title    "Monitor"
       :bg-color 0xFFFFFFFF
       :exit-on-close? false}
      *app)))

(comment
  (app/doui-async
    (open-window!))
  )