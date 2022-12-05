(ns idp.robot.brain.util
  "Common functions and phases used throughout the autopilot"
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [clojure.core.match :refer [match]]
    [idp.robot.state :as rs]))

(defn motor-input
  "Convenience function for expressing a motor speed input in
  terms of the superposition of a forwards speed and a clockwise turning speed"
  ([forward-speed] (motor-input forward-speed 0))
  ([forward-speed clockwise-turn-speed]
   {:motor-1 (+ forward-speed clockwise-turn-speed)
    :motor-2 (- forward-speed clockwise-turn-speed)}))

(defn clamp-motor-speed [s]
  (min 255 (max -255 s)))

(defn calc-motor-turn-amount
  "Estimates, based on motor input, the amount by
  which the robot has turned clockwise since previous response"
  [{:keys [readings input]}]
  (let [turn-speed (- (clamp-motor-speed (:motor-1 input))
                     (clamp-motor-speed (:motor-2 input)))]
    (* turn-speed (rs/get-active-dt readings))))

(defn calc-motor-forward-amount
  "Estimates, based on motor input, the amount by
  which the robot has moved forwards since previous response"
  [{:keys [readings input]}]
  (let [speed (+ (clamp-motor-speed (:motor-1 input))
                (clamp-motor-speed (:motor-2 input)))]
    (* speed (rs/get-active-dt readings))))

(defphase stop
  "While in this phase, set motor speed to zero"
  :tick (fn [_] {:input (motor-input 0)}))

(defphase set-input
  "Sets the requested robot input according to the initial state parameters
  and completes immediately"
  :init (fn [params]
          {:input (select-keys params
                    [:motor-1 :motor-2 :ultrasonic-active?
                     :signal-block-density :grabber-position])})
  :tick (fn [{{:keys [input]} :state}]
          (phase/mark-done {:input input})))

(defn get-line-triggers
  "Line trigger = number of times line sensor has switched
  from seeing black to seeing white since the last response.
  Returns a 4-tuple of integers."
  [{:keys [line-switches] :as readings}]
  {:pre [(map? readings)]}
  (let [line-sensor-readings (rs/get-line-sensors readings)]
    (mapv (fn [n]
            (let [white? (= :white (nth line-sensor-readings n))
                  nswitches (nth line-switches n)
                  nswitches-up-to-white (max 0 (cond-> nswitches (not white?) dec))]
              (quot (inc nswitches-up-to-white) 2)))
      (range 0 4))))

(defn get-combined-line-readings
  "Results of whether each line sensor saw white at least once
  since last response.
  Returns 4-tuple of :w or :b"
  [readings]
  (let [line-triggers (get-line-triggers readings)]
    (mapv #(if (or (= :white %1) (pos? %2)) :w :b)
      (rs/get-line-sensors readings)
      line-triggers)))

(defphase tracking-prolonged-condition
  "Completes only when a specified condition has been satisfied
  for a certain contiguous duration"
  :init (fn [{:keys [min-duration pred]}]
          {;; determines when phase is done (ms)
           :min-duration (enc/have integer? min-duration)
           ;; condition that must be held for the specified duration
           :pred (enc/have fn? pred)
           
           :satisfied-duration 0})
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [satisfied? ((:pred state) robot)]
      (if (<= (:min-duration state) (:satisfied-duration state))
        (phase/mark-done {})
        {:state {:satisfied-duration
                 (if satisfied?
                   (+ (:satisfied-duration state) (rs/get-active-dt readings))
                   0)}}))))

(defphase straight-up-to-blackout
  "Goes straight until line no longer seen.
  Requires a certain number of 'blackout' readings to end."
  :init {:blackout-duration 0}
  :tick
  (fn [{:keys [state readings]}]
    (let
      [min-blackouts-duration 200 ;; determines when phase is done (ms)
       blackout-speed 50 ;; after first blackout, go slower
       
       combined-readings (get-combined-line-readings readings)
       {:keys [blackout-duration]} state
       max-fspeed (if (pos? blackout-duration) blackout-speed 200)
       forward-speed max-fspeed]
      (if (<= min-blackouts-duration blackout-duration)
        (phase/mark-done {})
        {:state {:blackout-duration
                 (if (= [:b :b :b :b] combined-readings)
                   (+ blackout-duration (rs/get-active-dt readings))
                   0)}
         :input (motor-input forward-speed 0)}))))

(defphase timed-straight
  "Move straight forwards (or backwards) for fixed amount of time at a given speed"
  :init (fn [params]
          {:duration (:duration params)
           :speed (:speed params 200)
           :turn-speed (:turn-speed params 0)
           :time-elapsed 0})
  :tick
  (fn [{{:keys [speed] :as state} :state
        :keys [readings]}]
    (let [time-elapsed (+ (:time-elapsed state)
                         (rs/get-active-dt readings))]
      (if (<= (:duration state) time-elapsed)
        (phase/mark-done {})
        {:state {:time-elapsed time-elapsed}
         :input (motor-input speed (:turn-speed state))}))))

(defphase tracking-motor-turn
  "Estimates, based on motor input, the amount by
  which the robot has turned clockwise"
  :init {:turn-amount 0}
  :tick
  (fn [{:keys [state] :as robot}]
    {:state {:turn-amount (+ (:turn-amount state)
                            (calc-motor-turn-amount robot))}}))

(defphase tracking-motor-forward
  "Estimates, based on motor input, the amount by
  which the robot has moved forwards"
  :init {:forward-amount 0}
  :tick
  (fn [{:keys [state] :as robot}]
    {:state {:forward-amount (+ (:forward-amount state)
                               (calc-motor-forward-amount robot))}}))

(defphase junction-turn-spin
  "Turn from one junction exit to another.
  At the end, line sensors should be centred on the line."
  :init (fn [{:keys [turn-direction] :as params}]
          {:location :start-line ;; :start-line, :between, :end-line
           :forward-speed (:forward-speed params 0)
           :turn-direction turn-direction})
  :tick
  (fn [{:keys [state readings]}]
    (let [{:keys [forward-speed]} state
          turn-speed 160
          left? (= :left (:turn-direction state))
          turn-speed (cond-> turn-speed left? -)
          combined-line-readings
          (cond-> (get-combined-line-readings readings)
            left? (-> rseq vec))
          ;; For clockwise turn
          location (:location state)
          [done? location]
          (if (= :start-line location)
            [false
             (match combined-line-readings
               [:w _ _ _] :between
               [:b :b :b :b] :between
               :else :start-line)]
            (match combined-line-readings
             [_  :w _  _]
             [(= :end-line location) :between]
             [:b _ _ :w]
             [false :end-line]
             :else
             [false location]))]
      (if done?
        (phase/mark-done {})
        {:state {:location location}
         :input (motor-input forward-speed turn-speed)}))))

(defphase tracking-motor-turn-rate
  "Calculates the turn rate over a fixed duration
  based on motor input"
  :init (fn [params]
          {:target-duration (:target-duration params 700) ;; ms
           
          :history (enc/queue)
          :duration 0 ;; ms
          :turn-amount 0
          :turn-rate 0 ;; turn amount per active millisecond
          })
  :sub-phases
  {:turn [tracking-motor-turn]}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [turn-cmd (phase/tick-subphase robot :turn)
          Δturn-amount (-> turn-cmd :state :sub-phases :turn :turn-amount)
          dt (long (rs/get-active-dt readings))
          snapshot {:dt dt
                    :turn-amount Δturn-amount}
          duration (+ (:duration state) dt)
          turn-amount (+ (:turn-amount state) Δturn-amount)
          history (conj (:history state) snapshot)
          {:keys [target-duration]} state
          ;; Remove old snapshots
          [history duration turn-amount]
          (loop [history history
                 duration duration
                 turn-amount turn-amount]
            (if (not-empty history)
              (let [snapshot (peek history)
                    duration' (- duration (:dt snapshot))]
                (if (<= target-duration duration')
                  (recur (pop history)
                    duration'
                    (- turn-amount (:turn-amount snapshot)))
                  [history duration turn-amount]))
              [history duration turn-amount]))
          turn-rate (/ turn-amount (float duration))]
      {:state {:turn-rate turn-rate
               :history history
               :duration duration
               :turn-amount turn-amount}})))

(defphase until-straight
  "Infers, based on motor input, when robot starts
  going approximately straight.
  The turn rate must be within a certain range for a certain duration"
  :init (fn [params]
          {:max-turn-rate (:max-turn-rate params 5)})
  :sub-phases
  (fn [params]
    {:turn-rate [tracking-motor-turn-rate
                 {:target-duration (:min-straight-duration params 800)}]})
  :tick
  (fn [{:keys [state] :as robot}]
    (let [cmd (phase/tick-subphase robot :turn-rate)
          turn-rate-state (-> cmd :state :sub-phases :turn-rate)
          turn-rate (:turn-rate turn-rate-state)
          done? (and (<= (:target-duration turn-rate-state) (:duration turn-rate-state))
                  (<= (abs turn-rate) (:max-turn-rate state)))]
      (if done?
        (phase/mark-done cmd)
        cmd))))

(defphase until-turning
  "Infers, based on motor input, when robot starts turning.
  The turn rate must be greater than a certain amount for a certain duration"
  :init
  (fn [{:keys [turn-direction min-turn-rate]}]
    {:min-turn-rate (or min-turn-rate 17) ;; turn amount per active millisecond
     :turn-direction (enc/have #{:left :right :either} turn-direction)})
  :sub-phases
  {:turn [tracking-motor-turn-rate {:target-duration 700}]}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [cmd (phase/tick-subphase robot :turn)
          {:keys [turn-rate duration target-duration]}
          (-> cmd :state :sub-phases :turn)
          desired-direction? (case (:turn-direction state)
                               :left (neg? turn-rate)
                               :right (pos? turn-rate)
                               :either true)]
      (if (and (<= target-duration duration)
            desired-direction?
            (<= (:min-turn-rate state) (abs turn-rate)))
        (phase/mark-done cmd)
        cmd))))

(defphase position-grabber
  "Sets the grabber position and completes once the grabber is done moving."
  :init (fn [{:keys [grabber-position]}]
          {:started? false
           :grabber-position grabber-position})
  :tick
  (fn [{:keys [state readings]}]
    (let [input {:grabber-position (:grabber-position state)}]
      (if (:started? state)
        (if (not (:grabber-moving? readings))
          (phase/mark-done {})
          {:input input})
        {:state {:started? true}
         :input input}))))