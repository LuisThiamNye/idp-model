(ns idp.common
  (:require
    [io.github.humbleui.core :as core]
    [io.github.humbleui.font :as font]
    [io.github.humbleui.paint :as paint]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window])
  (:import
    
    (java.lang AutoCloseable)))

(defn with-context
  ([child]
   (with-context {} child))
  ([opts child]
   (ui/with-bounds ::bounds
     (ui/dynamic ctx [scale      (:scale ctx)
                      cap-height (or (some-> (:cap-height opts) (* scale))
                                   (-> ctx ::bounds :height (* scale) (quot 30)))]
       (let [#_#_#_#_#_#_font-body (font/make-with-cap-height resources/typeface-regular cap-height)
             font-h1   (font/make-with-cap-height resources/typeface-bold    cap-height)
             font-code (font/make-with-cap-height resources/typeface-code    cap-height)]
         (ui/default-theme {#_#_:face-ui resources/typeface-regular}
           (ui/with-context
             (merge
               {;:face-ui   resources/typeface-regular
                ; :font-body font-body
                ; :font-h1   font-h1
                ; :font-code font-code
                :leading   (quot cap-height 2)
                :fill-text (paint/fill 0xFF212B37)
                :unit      (quot cap-height 10)}
               opts)
             child)))))))