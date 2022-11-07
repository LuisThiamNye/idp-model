(ns idp.telnet.api
  (:require
    [clojure.data :as data]
    [idp.telnet :as telnet]))

(defn frac->byte [n]
  (unchecked-int (Math/round (* 255. n))))

(defn msg-motor-speed ^String [n speed]
  (assert (<= 0 speed 255))
  (assert (<= 1 n 2))
  (let [mul (Math/floor (/ speed 64))
        mag (- speed (* mul 64))]
    (str "m" n (char (+ 40 mag)) (char (+ 40 mul)))))

(defn set-motor-speed [n speed]
  (telnet/send-line! (msg-motor-speed n speed)))

; (defn reset-state!)

(def *state
  (atom {:motor-1 0
         :motor-2 0}))

(defn set-state [state]
  (let [[_ news _] (data/diff @*state state)
        sb (StringBuilder.)]
    (run!
      (fn [[k v]]
        (.append sb 
          (case k
            :motor-1
            (msg-motor-speed 1 v)
            :motor-2
            (msg-motor-speed 2 v))))
      news)
    (let [s (str sb)]
      (println "Sending:" (mapv int s))
      (when (< 0 (count s))
        (telnet/send-line! s)))
    (reset! *state state)))

(comment
  (set-motor-speed 1 255)
  (msg-motor-speed 1 0)
  
  (set-state
    {:motor-1 0
     :motor-2 0})
  
  (set-state
    {:motor-1 0
     :motor-2 0})
  
  (char 255)
  (Character/toString 255)
  
  (let [speed 128
        extended? (< 127 speed)
        s (if extended?
            (- speed 128) speed)]
    (char (if extended? 1 0))
    )
  
  )