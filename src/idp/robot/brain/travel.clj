(ns idp.robot.brain.travel
  (:require
    [idp.telnet.api :as api]
    [clojure.core.match :refer [match]]
    [idp.robot.command :as cmd]
    [idp.robot.state :as robot.state]))

(def *state
  (atom {:mode :exit-start
         :over-horiz? false
         :n-horiz-found 0
         :deviation :none}))

(defn tick-stop []
  (swap! api/*input assoc
    :motor-1 0 :motor-2 0))

(defn handle-deviation!
  [{:keys [turn-speed straight-speed turn-diff]}]
  (let [{:keys [deviation]} @*state]
    (case deviation
      :left
      (swap! api/*input assoc
        :motor-1 (+ turn-speed turn-diff)
        :motor-2 (+ turn-speed (- turn-diff)))
      :none
      (swap! api/*input assoc
        :motor-1 straight-speed
        :motor-2 straight-speed)
      :right
      (swap! api/*input assoc
        :motor-1 (+ turn-speed (- turn-diff))
        :motor-2 (+ turn-speed turn-diff)))))

(defn tick-simple-follow []
  (let [{:keys [line-sensor-1 ;; far right
                line-sensor-2 ;; centre right
                line-sensor-3 ;; centre left
                line-sensor-4 ;; far left
                ]} @api/*state
        line-sensor-left line-sensor-3
        line-sensor-centre line-sensor-2
        line-sensor-right line-sensor-1]
    (match [line-sensor-left
            line-sensor-centre
            line-sensor-right]
      [_ true _]
      (swap! *state assoc :deviation :none)
      [true false false]
      (swap! *state assoc :deviation :right)
      [false false true]
      (swap! *state assoc :deviation :left)
      :else nil)
    (handle-deviation!
      {:turn-speed 30
       :turn-diff 70
       :straight-speed 250})))

(defn tick-tight-follow []
  (let
    [{:keys [line-sensor-1 ;; far right
             line-sensor-2 ;; centre right
             line-sensor-3 ;; centre left
             line-sensor-4 ;; far left
             ]} @api/*state
     line-sensor-far-left line-sensor-4
     line-sensor-left line-sensor-3
     line-sensor-right line-sensor-2
     line-sensor-far-right line-sensor-1
     _
     (match [line-sensor-far-left
             line-sensor-left
             line-sensor-right
             line-sensor-far-right]
       [_ true true _]
       (swap! *state assoc :deviation :none)
       [true _ _ true]
       (swap! *state assoc :deviation :none)
       [_ true false false]
       (swap! *state assoc :deviation :right)
       [true false false false]
       (swap! *state assoc :deviation :right)
       [false false true _]
       (swap! *state assoc :deviation :left)
       [false false false true]
       (swap! *state assoc :deviation :left)
       :else nil)]
    ; (when (or line-sensor-far-left
    ;         line-sensor-far-right)
    ;   (swap! *state assoc :deviation-far? true))
    ; (when (or line-sensor-left
    ;         line-sensor-right)
    ;   (swap! *state assoc :deviation-far? false))
    (swap! *state assoc :deviation-far?
      (not (or line-sensor-left
             line-sensor-right)))
    ; (prn (select-keys @*state [:deviation :deviation-far?]))
    (handle-deviation!
      (if (:deviation-far? @*state)
        {:turn-speed 30
         :turn-diff 70
         :straight-speed 250}
        {:turn-speed 250
         :turn-diff 30
         :straight-speed 250}))))

(defn straight-follow-intent []
  (let
    [{:keys [line-sensor-1 ;; far right
             line-sensor-2 ;; centre right
             line-sensor-3 ;; centre left
             line-sensor-4 ;; far left
             ]} @api/*state
     line-sensor-far-left line-sensor-4
     line-sensor-left line-sensor-3
     line-sensor-right line-sensor-2
     line-sensor-far-right line-sensor-1
     [direction line-side deviation-far?]
     (match [line-sensor-far-left
             line-sensor-left
             line-sensor-right
             line-sensor-far-right]
       [_ true true _]
       [:straight nil false]
       [true _ _ true]
       [:straight nil]
       [_ true false false]
       [:straight :left false]
       [true false false false]
       [:left :right true]
       [false false true _]
       [:straight :right false]
       [false false false true]
       [:right :left true]
       [false false false false]
       [nil nil nil]
       :else nil)]
    (let [line-side (or line-side (:line-side @*state))
          direction (or direction (when (:deviation-far? @*state) line-side) :straight)
          deviation (case direction
                      :right :left
                      :left :right
                      :straight :none)]
      
      (swap! *state assoc
        :line-side line-side
        :deviation-far? (when deviation-far?
                          deviation-far?
                          (:deviation-far? @*state))
        :deviation deviation))
    (prn (select-keys @*state [:deviation :deviation-far?]))
    (handle-deviation!
      (if true ; deviation-far?
        {:turn-speed 30
         :turn-diff 70
         :straight-speed 150}
        {:turn-speed 150
         :turn-diff 30
         :straight-speed 150}))))

(defn tick-enter-tunnel []
  (when-not (:ultrasonic-active? @api/*input)
    (swap! api/*state assoc
      :ultrasonic-1 0 :ultrasonic-2 0))
  (swap! api/*input assoc
    :ultrasonic-active? true)
  (let [{:keys [ultrasonic-1
                ultrasonic-2]} @api/*state]
    (prn ultrasonic-1)
    (if (< 0 ultrasonic-1 300)
      (swap! *state assoc :mode :through-tunnel)
      (straight-follow-intent))))

(defn tick-through-tunnel [])

(defn tick-start-exit-turn []
  (let [{:keys [line-sensor-1 ;; far right
                line-sensor-2 ;; centre right
                line-sensor-3 ;; centre left
                line-sensor-4 ;; far left
                ]} @api/*state
        ; line-sensor-left line-sensor-4
        ; line-sensor-centre line-sensor-3
        line-sensor-right line-sensor-2
        line-sensor-far-right line-sensor-1]
    #_(match [line-sensor-left
            line-sensor-centre
            line-sensor-right]
      [false true false]
      (swap! *state assoc :deviation :none)
      [true false false]
      (swap! *state assoc :deviation :right)
      [false false true]
      (swap! *state assoc :deviation :left)
      :else nil)
    (if (and
          (not (:over-horiz? @*state))
          (or line-sensor-far-right
            line-sensor-right))
      (do
        (swap! *state assoc :mode :simple-follow)
        (tick-simple-follow))
      (let [;{:keys [deviation]} @*state
           sturn 80
           sturnf 120]
        (when-not (or line-sensor-far-right
                    line-sensor-right)
          (swap! *state assoc :over-horiz? false))
        (swap! api/*input assoc
          :motor-1 (+ sturnf sturn)
          :motor-2 (- sturnf sturn))))))

(defn tick-exit-start []
  (let
    [sforward 100
     {:keys [line-sensor-1
             line-sensor-2
             line-sensor-3
             line-sensor-4]} @api/*state
     {:keys [over-horiz?
             n-horiz-found]} @*state
     horiz? (<= 3
              (reduce +
                (mapv #(if % 1 0)
                  [line-sensor-1
                   line-sensor-2
                   line-sensor-3
                   line-sensor-4])))]
    (match [horiz? over-horiz?]
      [true true]
      nil
      [true false] ;; entered horiz
      (if (<= 1 n-horiz-found)
        ;; found the T-junction
        (do (println "STOP")
          (swap! *state assoc
            :mode :exit-start-turn
            :over-horiz? true
            :deviation :left)
          (tick-stop))
        ;; found the box edge
        (swap! *state assoc
          :over-horiz? true
          :n-horiz-found (inc n-horiz-found)))
      [false true] ;; exit horiz
      (swap! *state assoc :over-horiz? false)
      [false false]
      nil)
    (when (= :exit-start (:mode @*state))
      (let [speed sforward]
       (swap! api/*input assoc
         :motor-1 speed
         :motor-2 speed)))))

(defn tick! []
  (let [mode (:mode @*state)]
    (case mode
      :exit-start
      (tick-exit-start)
      :exit-start-turn
      (tick-start-exit-turn)
      :simple-follow
      (tick-simple-follow)
      :tight-follow
      (tick-tight-follow)
      :enter-tunnel
      (tick-enter-tunnel)
      :through-tunnel
      (tick-stop)
      :stop
      (tick-stop)
      (println "INVALID MODE!! " mode))))



