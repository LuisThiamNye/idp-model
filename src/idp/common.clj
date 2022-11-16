(ns idp.common
  (:require
    [clojure.string :as str]
    [io.github.humbleui.core :as core]
    [io.github.humbleui.font :as font]
    [io.github.humbleui.paint :as paint]
    [io.github.humbleui.protocols :as protocols]
    [io.github.humbleui.ui :as ui]
    [io.github.humbleui.window :as window])
  (:import
    (io.github.humbleui.skija
      Typeface FontStyle FontWeight FontWidth FontSlant)
    (java.lang AutoCloseable)))

(defmacro future-virtual [& body]
  `(let [p# (promise)]
     (.start (Thread/ofVirtual)
       (fn [] (deliver p# (do ~@body))))
     p#))

(def code-typeface (Typeface/makeFromName "Input Mono"
                        (FontStyle.
                          FontWeight/NORMAL
                          FontWidth/EXTRA_CONDENSED
                          FontSlant/UPRIGHT)))

(def mono-typeface code-typeface)

(defn multiline-label [string]
  (ui/column
    (map #(ui/padding 0 2
            (ui/label %))
      (str/split-lines string))))

(defn with-context
  ([child]
   (with-context {} child))
  ([opts child]
   (ui/with-bounds ::bounds
     (ui/dynamic ctx
       [scale      (:scale ctx)
        cap-height (or (some-> (:cap-height opts) (* scale))
                     (-> ctx ::bounds :height (* scale) (quot 30)))
        self-reload @#'with-context]
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
                :unit      (quot cap-height 10)
                :hui.button/padding-left 5
                :hui.button/padding-top 4
                :hui.button/padding-right 5
                :hui.button/padding-bottom 4}
               opts)
             child)))))))