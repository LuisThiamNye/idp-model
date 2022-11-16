(ns idp.robot.brain.travel
  (:require
    [idp.net.api :as api]
    [clojure.core.match :refer [match]]
    [idp.robot.state :as robot.state]))

(defn tick-stop []
  {:motor-1 0 :motor-2 0})

(defn handle-deviation!
  [*state {:keys [turn-speed straight-speed turn-diff]}]
  (let [{:keys [deviation]} @*state]
    (case deviation
      :left
      {:motor-1 (+ turn-speed turn-diff)
       :motor-2 (+ turn-speed (- turn-diff))}
      :none
      {:motor-1 straight-speed
       :motor-2 straight-speed}
      :right
      {:motor-1 (+ turn-speed (- turn-diff))
       :motor-2 (+ turn-speed turn-diff)})))

(defn tick-simple-follow [*state readings]
  (let [{:keys [line-sensor-1 ;; far right
                line-sensor-2 ;; centre right
                line-sensor-3 ;; centre left
                line-sensor-4 ;; far left
                ]} readings
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
    (handle-deviation! *state
      {:turn-speed 30
       :turn-diff 70
       :straight-speed 250})))

(defn tick-tight-follow [*state readings]
  (let
    [{:keys [line-sensor-1 ;; far right
             line-sensor-2 ;; centre right
             line-sensor-3 ;; centre left
             line-sensor-4 ;; far left
             ]} readings
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
    (handle-deviation! *state
      (if (:deviation-far? @*state)
        {:turn-speed 30
         :turn-diff 70
         :straight-speed 250}
        {:turn-speed 250
         :turn-diff 30
         :straight-speed 250}))))

(defn straight-follow-intent [*state readings]
  (let
    [{:keys [line-sensor-1 ;; far right
             line-sensor-2 ;; centre right
             line-sensor-3 ;; centre left
             line-sensor-4 ;; far left
             ]} readings
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
    (handle-deviation! *state
      (if true ; deviation-far?
        {:turn-speed 30
         :turn-diff 70
         :straight-speed 150}
        {:turn-speed 150
         :turn-diff 30
         :straight-speed 150}))))

(defn tick-enter-tunnel [*state readings]
  (let [{:keys [ultrasonic-1
                ultrasonic-2]} readings]
    (prn ultrasonic-1)
    (if (< 0 ultrasonic-1 300)
      (do (swap! *state assoc :mode :through-tunnel)
        {:ultrasonic-active? true})
      (straight-follow-intent *state readings))))

(defn tick-through-tunnel [*state readings])

(defn get-line-triggers
  "Line trigger = number of times line sensor has encountered
  a distinct white line since the last response."
  [{:keys [line-switches] :as readings}]
  (let [line-sensor-readings (robot.state/get-line-sensors readings)
        sensor-line-enters
        (fn [n]
          (let [on? (= :white (nth line-sensor-readings n))
                nswitches (nth line-switches n)
                n-ons (long (Math/ceil (/ (cond-> nswitches
                                            (not on?) dec)
                                         2)))]
            n-ons))]
    [(sensor-line-enters 0)
     (sensor-line-enters 1)
     (sensor-line-enters 2)
     (sensor-line-enters 3)]))

(defn update-line-triggers
  [line-triggers readings]
  (mapv + line-triggers (get-line-triggers readings)))

(def state0-tunnel-approach
  {:mode :tunnel-approach
   :follow-intent [:straight]})

(defn tick-tunnel-approach [*state readings]
  (let
    [line-triggers (get-line-triggers readings)
     ;; whether each line sensor saw white since last response
     combined-readings
     (-> (mapv #(if (or (= :white %1) (pos? %2)) :w :b)
           (robot.state/get-line-sensors readings)
           line-triggers)
       rseq vec)
     state @*state
     prev-intent (:follow-intent state)
     intent
     (match combined-readings
       [_ :w :w _] [:straight]
       [:b :w :b :b] [:left 1]
       [:w _ :b :b] [:left 2]
       [:b :b :w :b] [:right 1]
       [:b :b _ :w] [:right 2]
       [:b :b :b :b]
       (match prev-intent
         [:left 1]  [:straight]
         [:left 2]  [:left 3]
         [:right 1] [:straight]
         [:right 2] [:right 3]
         :else prev-intent)
       :else prev-intent)
     [forward-speed turn-speed]
     (match intent
       [:straight] [255 0]
       [left-or-right level]
       (let [[forward turn]
             (case (int level)
               1 [230 30]
               2 [20 200]
               3 [0 250])]
         [forward
          (cond-> turn (= :left left-or-right) -)]))]
    (swap! *state assoc :follow-intent intent)
    {:motor-1 (+ forward-speed turn-speed)
     :motor-2 (- forward-speed turn-speed)}))

(def state0-basic-follow
  {:mode :basic-follow
   :follow-intent [:straight]})

(defn tick-basic-follow [*state readings]
  (let
    [line-triggers (get-line-triggers readings)
     ;; whether each line sensor saw white since last response
     combined-readings
     (-> (mapv #(if (or (= :white %1) (pos? %2)) :w :b)
           (robot.state/get-line-sensors readings)
           line-triggers)
       rseq vec)
     state @*state
     prev-intent (:follow-intent state)
     intent
     (match combined-readings
       [_ :w :w _] [:straight]
       [:b :w :b :b] [:left 1]
       [:w :b :b :b] [:left 3]
       [:b :b :w :b] [:right 1]
       [:b :b :b :w] [:right 3]
       [:b :b :b :b]
       (match prev-intent
         [:left 1]  [:left 2]
         [:left 3]  [:left 4]
         [:right 1] [:right 2]
         [:right 3] [:right 4]
         :else prev-intent)
       :else prev-intent)
     [forward-speed turn-speed]
     (match intent
       [:straight] [255 0]
       [left-or-right level]
       (let [[forward turn]
             (case (int level)
               1 [230 30]
               2 [80 150]
               3 [20 500]
               4 [0 250])]
         [forward
          (cond-> turn (= :left left-or-right) -)]))]
    (swap! *state assoc :follow-intent intent)
    {:motor-1 (+ forward-speed turn-speed)
     :motor-2 (- forward-speed turn-speed)}))

(def state0-start-exit-turn
  {:mode :exit-start-turn
   :line-triggers [0 0 0 0]})

(defn tick-start-exit-turn [*state readings]
  (let [sturn 120
        sturnf 170
        {:keys [line-sensor-1 ;; far right
                line-sensor-2 ;; centre right
                ; line-sensor-3 ;; centre left
                ; line-sensor-4 ;; far left
                ]} readings
        ; line-sensor-left line-sensor-4
        ; line-sensor-centre line-sensor-3
        line-sensor-right line-sensor-2
        line-sensor-far-right line-sensor-1
        {:keys [line-triggers]} @*state
        line-triggers (update-line-triggers line-triggers readings)
        _ (swap! *state assoc :line-triggers line-triggers)
        start-following?
        ;; start following when right two sensors have encountered the line
        (every? pos? (subvec line-triggers 0 2))]
    (if start-following?
      (do
        (swap! *state merge state0-basic-follow)
        (tick-basic-follow *state readings))
      (do
        (when-not (or line-sensor-far-right line-sensor-right)
          (swap! *state assoc :over-horiz? false))
        {:motor-1 (+ sturnf sturn)
         :motor-2 (- sturnf sturn)}))))

(def state0-exit-start
  {:mode :exit-start
   :line-triggers [0 0 0 0]})

(defn tick-exit-start [*state readings]
  (let
    [sforward 200
     {:keys []} readings
     {:keys [line-triggers] :as _state} @*state
     line-triggers (update-line-triggers line-triggers readings)
     _ (swap! *state assoc :line-triggers line-triggers)
     ;; one line sensor may remain on the path and not be triggered twice
     found-junction?
     (let [non-ones (filterv #(not= % 1) line-triggers)]
       (and (<= 3 (count non-ones))
         (every? #(<= 2 %) non-ones)))
     start-turning?
     (and found-junction?
       (every? #(= % :black) (robot.state/get-line-sensors readings)))]
    (if start-turning?
      (do
        (swap! *state merge state0-start-exit-turn)
        (tick-start-exit-turn *state
          (assoc readings :line-switches [0 0 0 0])))
      (let [speed sforward]
        {:motor-1 speed
         :motor-2 speed}))))

(defn tick! [*state readings]
  (let [mode (:mode @*state)]
    (case mode
      :exit-start
      (tick-exit-start *state readings)
      :exit-start-turn
      (tick-start-exit-turn *state readings)
      :basic-follow
      (tick-basic-follow *state readings)
      ; :simple-follow
      ; (tick-simple-follow *state readings)
      ; :tight-follow
      ; (tick-tight-follow *state readings)
      ; :enter-tunnel
      ; (tick-enter-tunnel *state readings)
      ; :through-tunnel
      ; (tick-stop)
      :tunnel-approach
      (tick-tunnel-approach *state readings)
      :stop
      (tick-stop)
      (println "INVALID MODE!! " mode))))
