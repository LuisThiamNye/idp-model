(ns idp.common
  "Miscellaneous functions used across the program"
  (:require
    [clojure.core.match :refer [match]]
    [clojure.string :as str]
    [io.github.humbleui.paint :as paint]
    [io.github.humbleui.ui :as ui])
  (:import
    (io.github.humbleui.skija
      Typeface FontStyle FontWeight FontWidth FontSlant)))

(defn on-agent
  "Returns the results of f applied to the agent.
  f runs on the agent, but result is returned synchronously"
  [agent f & args]
  (let [res (promise)]
    (send-off agent
      #(do (deliver res
             (try [:ok (apply f % args)]
               (catch Throwable e
                 [:error e])))
         %))
    (match @res
      [:ok ret] ret
      [:error e] (throw e))))

(def code-typeface (Typeface/makeFromName "Input Mono"
                        (FontStyle.
                          FontWeight/NORMAL
                          FontWidth/EXTRA_CONDENSED
                          FontSlant/UPRIGHT)))

(def mono-typeface code-typeface)

(defn action-checkbox
  "Options:
  :state → value for that checkbox state (boolean or :indeterminate)
  :on-toggle → function called with two args: previous and
  updated checkbox states."
  [{:keys [state on-toggle]} child]
  (ui/checkbox
    (reify
      clojure.lang.IDeref
      (deref [_]
        state)
      clojure.lang.IAtom
      (swap [_ f]
        (on-toggle state (f state))))
    child))

(defn multiline-label [string]
  (ui/column
    (map #(ui/padding 0 2
            (ui/label %))
      (str/split-lines string))))

(defn with-context
  "Injects common context that may be useful"
  ([child]
   (with-context {} child))
  ([opts child]
   (ui/with-bounds ::bounds
     (ui/dynamic _ctx []
       (ui/default-theme {}
         (ui/with-context
           (merge
             {:fill-text (paint/fill 0xFF212B37)
              :hui.button/padding-left 5
              :hui.button/padding-top 4
              :hui.button/padding-right 5
              :hui.button/padding-bottom 4}
             opts)
           child))))))