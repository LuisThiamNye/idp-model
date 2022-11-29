(ns idp.robot.brain.travel
  (:require
    [taoensso.encore :as enc]
    [idp.board.params :as board.params]
    [idp.robot.params :as robot.params]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [clojure.core.match :refer [match]]
    [idp.robot.state :as robot.state]))

(defn motor-input
  ([forward-speed] (motor-input forward-speed 0))
  ([forward-speed clockwise-turn-speed]
   {:motor-1 (+ forward-speed clockwise-turn-speed)
    :motor-2 (- forward-speed clockwise-turn-speed)}))

(defphase stop
  :init {}
  :tick (fn [_] {:input (assoc (motor-input 0)
                          :signal-block-density nil)}))

(defn get-line-triggers
  "Line trigger = number of times line sensor has switched
  from seeing black to seeing white since the last response."
  [{:keys [line-switches] :as readings}]
  (let [line-sensor-readings (robot.state/get-line-sensors readings)]
    (mapv (fn [n]
            (let [white? (= :white (nth line-sensor-readings n))
                  nswitches (nth line-switches n)
                  nswitches-up-to-white (max 0 (cond-> nswitches (not white?) dec))]
              (quot (inc nswitches-up-to-white) 2)))
      (range 0 4))))

(defn update-line-triggers [line-triggers readings]
  (mapv + line-triggers (get-line-triggers readings)))

(defn get-combined-line-readings
  "Results of whether each line sensor saw white at least once
  since last response.
  Returns 4-tuple of :w or :b"
  [readings]
  (let [line-triggers (get-line-triggers readings)]
    (mapv #(if (or (= :white %1) (pos? %2)) :w :b)
      (robot.state/get-line-sensors readings)
      line-triggers)))

(defn tick-common-follow [{:keys [state readings]} follow-strategy]
  (let [intent ((:intent-fn follow-strategy) state readings)]
    (if (= [:continue] intent)
     {}
     (let [level-speeds (:level-speeds follow-strategy)
           [forward-speed turn-speed]
           (match intent
             [:straight] (nth level-speeds 0)
             [left-or-right level]
             (let [[forward turn] (nth level-speeds level)]
               [forward
                (cond-> turn (= :left left-or-right) -)]))]
       {:state {:follow-intent intent}
        :input (motor-input forward-speed turn-speed)}))))

(def basic-follow-strategy
  {:intent-fn
   (fn [state readings]
     (let [combined-readings (get-combined-line-readings readings)
           prev-intent (:follow-intent state)]
       (match combined-readings
         [_ :w :w _] [:straight]
         [:b :w :b :b] [:left 1]
         [:w :w :b :b] [:left 2]
         [:w :b :b :b] [:left 3]
         [:b :b :w :b] [:right 1]
         [:b :b :w :w] [:right 2]
         [:b :b :b :w] [:right 3]
         [:b :b :b :b]
         (match prev-intent
           ;; go straight if line lost when between sensors
           ;; - important for ramp
           [:left 1]  [:straight]; [:left 2]
           [:left 3]  [:left 4]
           [:right 1] [:straight];[:right 2]
           [:right 3] [:right 4]
           :else prev-intent)
         :else prev-intent)))})

(defphase basic-follow
  "Follow the line, centred on the robot"
  :init {:follow-intent [:continue]
         :high-power? false}
  :tick
  (fn [{:keys [state] :as robot}]
    (tick-common-follow robot
      (assoc basic-follow-strategy :level-speeds
        (if (:high-power? state)
          [[255 0] [255 30] [255 30] [0 255] [0 255]]
          [[255 0] [230 30] [80 150] [20 500] [0 250]])))))

(def biased-follow-strategy
  {:intent-fn
   (fn [state readings]
     (let [combined-readings (get-combined-line-readings readings)
           mirror-intent #(update % 0
                            (some-fn
                              {:left :right
                               :right :left}
                              identity))
           ;; Calculate in terms of left bias
           swap? (case (:bias state) :right true :left false)
           prev-intent (cond-> (:follow-intent state [:straight])
                         swap? mirror-intent)
           intent
           (match (cond-> combined-readings
                    swap? (-> rseq vec))
             [ _ :w :w :w] [:straight]
             [:b :b :w  _] [:straight]
             [ _ :w :w :b] [:left 1]
             [ _ :w :b :b] [:left 2]
             [:w :b :b :b] [:left 2]
             [ _  _ :b :w] [:right 1]
             [:b :b :b :b]
             (match prev-intent
               [:right 1] [:right 2]
               :else prev-intent)
             :else prev-intent)]
       (cond-> intent swap? mirror-intent)))})

(defphase biased-follow
  "Follow the line, keeping the line aligned with either the
  middle- left or right sensor"
  :init (fn [{:keys [bias]}]
          {:follow-intent [:continue]
           :bias (enc/have #{:left :right} bias)
           :high-power? false})
  :tick
  (fn [{:keys [state] :as robot}]
    (tick-common-follow robot
      (assoc biased-follow-strategy :level-speeds
        (if (:high-power? state)
          [[255 0] [255 30] [0 255]]
          [[255 0] [255 30] [0 200]])))))

(defphase tracking-prolonged-condition
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
                   (+ (:satisfied-duration state) (robot.state/get-active-dt readings))
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
                   (+ blackout-duration (robot.state/get-active-dt readings))
                   0)}
         :input (motor-input forward-speed 0)}))))

(defphase follow-up-to-blackout
  "Does line following to tightly follow the line until it suddenly ends.
  Requires a certain number of 'blackout' readings to end.
  Assumes there is no 'dead zone' between any of the line sensors."
  :init {:follow-intent [:continue]
         :blackout-duration 0}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let
      [min-blackouts-duration 400 ;; determines when phase is done (ms)
       blackout-speed 150 ;; after first blackout, go slower
       combined-readings (get-combined-line-readings readings)
       {:keys [blackout-duration]} state
       max-fspeed (if (pos? blackout-duration) blackout-speed 255)
       cmd (tick-common-follow robot
             {:level-speeds [[max-fspeed 0]
                            [(* max-fspeed 0.9) 30]
                            [0 200]]
              :intent-fn
              (fn [state _readings]
                (let [prev-intent (:follow-intent state)]
                  (match combined-readings
                    [_ :w :w _] [:straight]
                    [:b :w :b :b] [:left 1]
                    [:b :b :w :b] [:right 1]
                    [:w _ :b :b] [:left 2]
                    [:b :b _ :w] [:right 2]
                    [:b :b :b :b]
                    (match prev-intent
                      [:left 1]  [:straight]
                      [:right 1] [:straight]
                      :else prev-intent)
                    :else prev-intent)))})]
      (if (<= min-blackouts-duration blackout-duration)
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          {:state {:blackout-duration
                   (if (= [:b :b :b :b] combined-readings)
                     (+ blackout-duration (robot.state/get-active-dt readings))
                     0)}})))))

(defphase timed-forwards
  "Move straight forwards (or backwards) for fixed amount of time at a given speed"
  :init (fn [params]
          {:duration (:duration params)
           :speed (:speed params 200)
           :turn-speed 0
           :time-elapsed 0})
  :tick
  (fn [{{:keys [speed] :as state} :state
        :keys [readings]}]
    (let [time-elapsed (+ (:time-elapsed state)
                         (robot.state/get-active-dt readings))]
      (if (<= (:duration state) time-elapsed)
        (phase/mark-done {})
        {:state {:time-elapsed time-elapsed}
         :input (motor-input speed (:turn-speed state))}))))

(defn clamp-motor-speed [s]
  (min 255 (max -255 (abs s))))

(defn calc-motor-turn-amount
  "Estimates, based on motor input, the amount by
  which the robot has turned clockwise since previous response"
  [{:keys [readings input]}]
  (let [turn-speed (- (clamp-motor-speed (:motor-1 input))
                     (clamp-motor-speed (:motor-2 input)))]
    (* turn-speed (robot.state/get-active-dt readings))))

(defn calc-motor-forward-amount
  "Estimates, based on motor input, the amount by
  which the robot has moved forwards since previous response"
  [{:keys [readings input]}]
  (let [speed (+ (clamp-motor-speed (:motor-1 input))
                (clamp-motor-speed (:motor-2 input)))]
    (* speed (robot.state/get-active-dt readings))))

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
          turn-speed 150
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
          dt (long (robot.state/get-active-dt readings))
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
  going approximately straight"
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
  "Infers, based on motor input, when robot starts turning"
  :init
  (fn [{:keys [turn-direction]}]
    {:min-turn-rate 17 ;; turn amount per active millisecond
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


(defn reattempt-collection?
  "After dropping off block, decides whether to try collecting another
  block or to go back home"
  [{:keys [competition-start-time collection-target] :as _merged-state}]
  (and collection-target
    (let [min-collect-duration (* 1.7 60 1000)
          competition-duration (* 5 60 1000) ;; 5 mins
          time-passed (- (System/currentTimeMillis) competition-start-time)
          time-remaining (- competition-duration time-passed)]
      (<= min-collect-duration time-remaining))))

(defphase decide-next-target
  "After having dropped off a block, decides where the robot should go next"
  :init {}
  :tick
  (fn [{:keys [merged-state]}]
    (phase/mark-done
      {:output {:next-target
                {:go-home? (not (reattempt-collection? merged-state))
                 :collection-target
                 (case (:collection-target merged-state)
                   2 3
                   3 1
                   1 nil)}}})))

(defphase junction-approach-turn-excess
  "Moves robot forwards, from position where line sensors are at the junction
  path, to where the centre of rotation is approximately aligned with
  the path of a box"
  :init (fn [{:keys [turn-direction] :as params}]
          {:status :pre-align
           :excess-coeff (:excess-coeff params 1)
           :turn-direction turn-direction})
  :sub-phases
  (fn [{:keys [turn-direction]}]
    {:forward-amount [tracking-motor-forward]
     :follow [biased-follow {:bias (case turn-direction
                                     :right :left
                                     :left :right)}]})
  :tick
  (fn [{:keys [readings]
        {:keys [status turn-direction excess-coeff]} :state
        :as robot}]
    (let [min-fwd-amount (long (* excess-coeff 70e4))
          cmd (phase/tick-subphase robot :forward-amount)
          forward-amount (-> cmd :state :sub-phases :forward-amount :forward-amount)]
      (if (<= min-fwd-amount forward-amount)
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          (phase/tick-subphase robot :follow)
          (when (= :pre-align status)
            ;; initially turn away from box a bit to avoid following the path up to it
            (let [ls (get-combined-line-readings readings)
                  nwhites (count (filterv #(= :w %) ls))
                  aligned? (<= nwhites 1)]
              (if aligned?
                {:state {:status :follow}}
                {:input (motor-input 0 (cond-> 200 (= :right turn-direction) -))}))))))))

(defphase junction-approach-turn
  :init {:status :excess}
  :sub-phases
  (fn [{:keys [turn-direction] :as params}]
    (enc/have? #{:left :right} turn-direction)
    {:excess [junction-approach-turn-excess
              {:turn-direction turn-direction
               :excess-coeff (:excess-coeff params 1)}]
     :spin [junction-turn-spin {:turn-direction turn-direction
                                :forward-speed (:turn-forward-speed params 0)}]})
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :excess
      (let [cmd (phase/tick-subphase robot :excess)]
        (if (phase/phase-done? cmd :excess)
          (update cmd :state assoc :status :spin)
          cmd))
      :spin
      (let [cmd (phase/tick-subphase robot :spin)]
        (if (phase/phase-done? cmd :spin)
          (phase/mark-done cmd)
          cmd)))))

(defphase up-to-junction
  "Goes from somewhere on the main line up to one of the
  three junctions of a line box or collection point.
  Performs biased line following to ensure junction detection."
  :init (fn [{:keys [junction-number turn-direction]}]
          {:junction-number (enc/have #{1 2 3} junction-number)
           :on-junction? false
           :nfinds 0
           :turn-direction turn-direction})
  :sub-phases
  (fn [{:keys [turn-direction]}]
    {:follow [biased-follow {:bias (enc/have #{:left :right} turn-direction)}]})
  :tick
  (fn [{:keys [state readings]
        {:keys [turn-direction]} :state :as robot}]
    (let [prev-on-junction? (:on-junction? state)
          combined-line-readings (get-combined-line-readings readings)
          ;; from perspective of turning right
          normalised-ls (cond-> combined-line-readings
                          (= :left (:turn-direction state))
                          (-> rseq vec))
          on-junction? (if prev-on-junction?
                          (match normalised-ls
                            [_ _ :b  :b] false
                            [:b :b :w :b] false
                            :else true)
                          (match normalised-ls
                            [_ _ :w :w] true
                            [:w :w :w _] true
                            :else false))
          nfinds (cond-> (:nfinds state) (and (not on-junction?)
                                           (not= prev-on-junction? on-junction?)) inc)
          {:keys [junction-number]} state
          cmd (cond-> (phase/tick-subphase robot :follow)
                ;; avoid following box path when at junction
                on-junction?
                (phase/merge-cmds
                  {:input (motor-input 180 (cond-> 80 (= :right turn-direction) -))}))]
      (if (<= junction-number nfinds)
        (phase/mark-done cmd)
        (phase/merge-cmds
          cmd
          {:state {:on-junction? on-junction?
                   :nfinds nfinds}})))))

(defphase up-to-dropoff-box
  "Goes from tunnel up to one of the three junctions of a line box.
  Performs right-biased line following to ensure junction detection."
  :init {:turn-excess-status :finding}
  :sub-phases
  (fn [{:keys [density]}]
    {:us-turning-condition [tracking-prolonged-condition
                            {:min-duration 80
                             :pred (fn [{:keys [readings]}]
                                     (<= 200 (:ultrasonic-2 readings) 500))}]
     :turn-excess [timed-forwards {:duration 1600 :speed 180}]
     :find-box [up-to-junction
                {:junction-number (case density
                                    :high 3 ;; red box
                                    :low 1 ;; green box
                                    (throw (ex-info "No density!" {})))
                 :turn-direction :right}]})
  :tick
  (fn [{{:keys [turn-excess-status]} :state :as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :find-box)
                (when (= :finding turn-excess-status)
                  (phase/merge-cmds
                    {:input {:ultrasonic-active? true}}
                    (phase/tick-subphase robot :us-turning-condition))))]
      (cond
        (phase/phase-done? cmd :find-box)
        (phase/mark-done cmd)
        
        (or (= turn-excess-status :doing)
          (and (= turn-excess-status :finding)
            (phase/phase-done? cmd :us-turning-condition)))
        (let [cmd (phase/merge-cmds cmd
                    {:status {:turn-excess-status :doing}}
                    (phase/tick-subphase robot :turn-excess))]
          (if (phase/phase-done? cmd :turn-excess)
            (phase/merge-cmds cmd
              {:state {:turn-excess-status :done}
               :input {:ultrasonic-active? false}})
            cmd))
        
        :else
        cmd))))

(defphase branch-to-main
  "Turns from collecting a block at a branch back onto the main line"
  :init {}
  :sub-phases
  {:turn [junction-turn-spin {:turn-direction :right}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/tick-subphase robot :turn)]
      (if (phase/phase-done? cmd :turn)
        (phase/mark-done cmd)
        cmd))))

(defphase collect-from-junction
  :init {:status :turn}
  :sub-phases
  {:turn [junction-approach-turn {:turn-direction :right
                                  :turn-forward-speed -150
                                  :excess-coeff 1.6}]
   :up-to-block [follow-up-to-blackout]}
  :tick
  (fn [{:keys [state]:as robot}]
    (case (:status state)
      :turn
      (let [cmd (phase/tick-subphase robot :turn)]
        (if (phase/phase-done? cmd :turn)
          (phase/merge-cmds cmd
            {:state {:status :up-to-block}})
          cmd))
      :up-to-block
      (let [cmd (phase/tick-subphase robot :up-to-block)]
        (if (phase/phase-done? cmd :up-to-block)
          (phase/mark-done cmd)
          cmd)))))

(defphase post-tunnel-to-collect
  "Goes from tunnel on the collect-side main line up to one of the
  junctions to collect a block.
  Performs biased line following to ensure junction detection."
  :init {}
  :sub-phases
  (fn [{:keys [collection-target]}]
    {:find-junction [up-to-junction
                     {:junction-number (enc/have #{1 3} collection-target)
                      :turn-direction :right}]})
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/tick-subphase robot :find-junction)]
      (if (phase/phase-done? cmd :find-junction)
        (phase/mark-done cmd)
        cmd))))

(defphase finalise-home
  :init {:status :forwards}
  :sub-phases
  {:forwards [timed-forwards {:duration 4000 :speed 255}]
   :backwards [timed-forwards {:duration 1700 :speed -100}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (let [status (:status state)
          cmd (phase/tick-subphase robot status)]
      (if (phase/phase-done? cmd status)
        (case status
          :forwards
          (phase/merge-cmds cmd
            {:state {:status :backwards}})
          :backwards
          (phase/mark-done cmd))
        cmd))))

#_(defphase align-to-home
  "Takes a robot on the home path up to the edge of the home box,
  aligning it so it is as straight as possible"
  :init {:sub-status :find-edge}
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :find-edge
      (let [found-edge?
            (match (get-combined-line-readings readings)
              [_ :w :w :w] true
              [:w :w :w _] true
              :else false)])
      )))

(defphase up-to-home-entry
  "Drives robot from a collection point up to the home junction,
  then spins it pointing towards the home box"
  :init {:status :find-turn}
  :sub-phases
  (fn [{:keys [density]}]
    (let [turn-direction (case density
                           :high :left
                           :low :right)]
      {:find-turn [up-to-junction {:turn-direction turn-direction
                                   :junction-number 1}]
       :turn [junction-approach-turn {:turn-direction turn-direction}]}))
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :find-turn
      (let [cmd (phase/tick-subphase robot :find-turn)]
        (if (phase/phase-done? cmd :find-turn)
          (phase/merge-cmds cmd
            {:state {:status :turn}})
          cmd))
      :turn
      (let [cmd (phase/tick-subphase robot :turn)]
        (if (phase/phase-done? cmd :turn)
          (phase/mark-done cmd)
          cmd)))))

(defphase backup-from-box-turn
  :init {}
  :sub-phases
  (fn [{:keys [go-home? density]}]
    {:turn [junction-turn-spin {:turn-direction
                                (if (enc/have boolean? go-home?)
                                  (case density
                                    :high :right
                                    :low :left)
                                  :right)}]})
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/tick-subphase robot :turn)]
      (if (phase/phase-done? cmd :turn)
        (phase/mark-done cmd)
        cmd))))

(defphase backup-from-box
  "Reverse from dropping off block at box"
  :init {:status :blackout-retreating}
  :sub-phases
  {:straight [timed-forwards {:duration 1100
                              :speed -255}]}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (case (:status state)
      :blackout-retreating
      (let [ls (get-combined-line-readings readings)]
        (if (match ls
              [_ :w _ _] true
              [_ _ :w _] true
              :else false)
          {:input (motor-input -255)}
          {:state {:status :retreating}}))
      :retreating
      (let [cmd (phase/tick-subphase robot :straight)]
        (if (phase/phase-done? cmd :straight)
          (phase/mark-done cmd)
          cmd)))))

(defphase box-approach-edge
  "Go up to edge of box"
  :init {:status :up-to-line}
  :sub-phases
  {:follow [straight-up-to-blackout]
   :excess [timed-forwards {:duration 300}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :up-to-line
      (let [cmd (phase/merge-cmds
                  (phase/tick-subphase robot :follow)
                  {:input {:grabber-position :open}})]
        (if (phase/phase-done? cmd :follow)
          (phase/merge-cmds cmd
            {:state {:status :excess}})
          cmd))
      :excess
      (let [cmd (phase/tick-subphase robot :excess)]
        (if (phase/phase-done? cmd :excess)
          (phase/mark-done cmd)
          cmd)))))

(defphase through-tunnel
  "Waits until refinding the line.
  Drives robot in straight line at constant speed.
  May use side-mounted ultrasonic sensors to adjust direction."
  :init {:nfinds 0
         :min-duration 1000
         :duration 0}
  :tick
  (fn [{:keys [state readings]}]
    (let [min-finds 3 ;; require multiple line encounters for extra certainty
          speed 255
          found-line? (and
                        (<= (:min-duration state) (:duration state))
                        (match (get-combined-line-readings readings)
                         [:w  _ :b :b] true
                         [ _ :w :b :b] true
                         [:b :w  _ :b] true
                         [:b  _ :w :b] true
                         [:b :b :w  _] true
                         [:b :b  _ :w] true
                         :else false))
          nfinds (cond-> (:nfinds state) found-line? inc)
          done? (<= min-finds nfinds)]
      (if done?
        (phase/mark-done {:input {:ultrasonic-active? false}})
        (let [;; ultrasonic-based correction algorithm as suggested by Greg
              turn-speed
              (let [max-x-error 10 ;; mm
                    correcting-turn-speed 100
                    ;; Use small angle approximations (robot going mostly straight)
                    us-x-offset (-> robot.params/dims :ultrasonic-1 :pos :x)
                    us-left-dist (:ultrasonic-1 readings)
                    deviation
                    (and (not (zero? us-left-dist))
                      (let [;; estimated horizontal position of robot relative to tunnel midpoint
                            x-pos (- us-left-dist
                                    us-x-offset (/ (:tunnel-width board.params/dims) 2))
                            abs-deviation (- (abs x-pos) max-x-error)]
                        (when (pos? abs-deviation)
                          (cond-> abs-deviation (neg? x-pos) -))))]
                (if deviation
                  (cond-> correcting-turn-speed (pos? deviation) -)
                  0))]
          {:state {:nfinds nfinds
                   :duration (+ (:duration state) (robot.state/get-active-dt readings))}
           :input (assoc (motor-input speed turn-speed)
                    :ultrasonic-active? true
                    :grabber-position :open)})))))

(defphase tunnel-approach
  "Does line following up to tunnel.
  Tunnel starts after observing a certain number of line sensor
  blackouts."
  :init {}
  :sub-phases
  {:follow [basic-follow]
   :follow-b [follow-up-to-blackout]
   :us-condition
   [tracking-prolonged-condition
    {:min-duration 100
     :pred
     (fn [{:keys [readings]}]
       (< 0 (:ultrasonic-1 readings) 150))}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :us-condition)
                {:input {:grabber-position :open
                         :ultrasonic-active? true}})]
      (if (phase/phase-done? cmd :us-condition)
        (let [cmd (phase/tick-subphase robot :follow-b)]
          (if (phase/phase-done? cmd :follow-b)
            (phase/mark-done
              (phase/merge-cmds cmd
                {:input {:ultrasonic-active? false}}))
            cmd))
        (phase/merge-cmds cmd
          (phase/tick-subphase robot :follow))))))

(defphase centre-block-180
  :init {:nturns 0}
  :sub-phases
  {:turn1 [timed-forwards nil {:duration 2000
                               :speed 0
                               :turn-speed 200}]
   :turn2 [junction-turn-spin nil {:turn-direction :right}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (let [nturns (:nturns state)
          first? (= 0 nturns)
          ss (if first? :turn1 :turn2)
          cmd (if first?
                (phase/tick-subphase robot :turn1)
                (phase/tick-subphase robot :turn2))]
      (if (phase/phase-done? cmd ss)
        (if first?
          (update cmd :state assoc :nturns (inc nturns))
          (phase/mark-done cmd))
        cmd))))

(defphase signal-block-density
  :init (fn [params]
          {:density (:density params :high)
           :time-elapsed 0})
  :tick
  (fn [{:keys [state readings]}]
    (let [signal-duration 5000
          time-elapsed (+ (:time-elapsed state) (:dt readings))]
      (if (<= signal-duration time-elapsed)
        (phase/mark-done
          {:input {:signal-block-density nil}})
        {:state {:time-elapsed time-elapsed}
         :input {:signal-block-density (:density state)}}))))

(defphase position-grabber
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

(defphase detect-block
  "Closes grabber, determines density, and signals the density.
  The grabber remains closed."
  :init {:status :pushing}
  :sub-phases
  {:position-grabber [position-grabber {:grabber-position :closed}]
   :pushing [timed-forwards {:duration 500}]
   :signal-block-density [signal-block-density]}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (case (:status state)
      :pushing
      (let [cmd (phase/tick-subphase robot :pushing)]
        (if (phase/phase-done? cmd :pushing)
          {:state {:status :closing}
           :input (motor-input 0)}
          (assoc-in cmd [:input :signal-block-density] nil)))
      :closing
      (let [cmd (phase/tick-subphase robot :position-grabber)]
        (if (phase/phase-done? cmd  :position-grabber)
          {:state {:status :detect}}
          cmd))
      :detect
      (let [density (:block-density readings)]
        {:state {:status :done-detect
                 :sub-phases
                 {:signal-block-density
                  {:density density}}}
         :output {:block-detected {:density density}}})
      :done-detect
      (let [cmd (phase/tick-subphase robot :signal-block-density)]
        (if (phase/phase-done? cmd :signal-block-density)
          (phase/mark-done cmd)
          cmd)))))

(defphase post-ramp-to-centre-block
  "Does left-biased line following up to central collection point.
  Conditions for stop:
  - Find the first collection junction (left turn, three line sensors)
  - Find the centre junction (three line sensors trigger on each side)"
  :init
  {:rhs-lines-found 0
   :lhs-lines-found 0}
  :sub-phases
  {:follow [biased-follow {:bias :right}]}
  :tick
  (fn [{:keys [state readings]
        {:keys [rhs-lines-found lhs-lines-found]} :state
        :as robot}]
    (let [combined-readings (get-combined-line-readings readings)
          [lhs-lines-found rhs-lines-found]
          (if (= combined-readings (:combined-line-readings state))
            [lhs-lines-found rhs-lines-found]
            (match combined-readings
              [:w :w :w :w]
              [(inc lhs-lines-found) (inc rhs-lines-found)]
              [:b :w :w :w]
              [lhs-lines-found (inc rhs-lines-found)]
              [:w :w :w :b]
              [(inc lhs-lines-found) rhs-lines-found]
              :else
              [lhs-lines-found rhs-lines-found]))
          done? (and (<= 1 rhs-lines-found)
                  (or (<= 2 lhs-lines-found)
                    (and (<= 1 lhs-lines-found)
                      (:block-present? readings))))]
      (if done?
        (phase/mark-done {})
        (phase/merge-cmds
          (phase/tick-subphase robot :follow)
          {:state {:lhs-lines-found lhs-lines-found
                   :rhs-lines-found rhs-lines-found
                   :combined-line-readings combined-readings}})))))

#_(defphase post-ramp-turning
  :init {}
  :sub-phases
  {:turn [tracking-motor-turn]}
  :tick
  (fn [{:as robot}]
    (let [;; after this turn amount, we have turned enough
          turn-amount-thres (long 3e5)
          turn-cmd (phase/tick-subphase robot :turn)
          done? (<= (-> turn-cmd :state :sub-phases :turn :turn-amount)
                  (- turn-amount-thres))]
      (if done?
        (phase/mark-done turn-cmd)
        turn-cmd))))

(defphase post-ramp-find-junction
  :init {:status :find-turn}
  :sub-phases
  {;:until-turning [until-turning {:turn-direction :left}]
   ; :turning [post-ramp-turning]
   :us-turning-condition [tracking-prolonged-condition
                          {:min-duration 80
                           :pred (fn [{:keys [readings]}]
                                   (<= 140 (:ultrasonic-2 readings) 500))}]
   :follow [biased-follow {:bias :left} {:high-power? false}]
   :until-straight [until-straight {:min-straight-duration 400
                                    :max-turn-rate 60}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :find-turn
      (let [cmd (phase/merge-cmds
                  ; (phase/tick-subphase robot :until-turning)
                  (phase/tick-subphase robot :follow)
                  (phase/tick-subphase robot :us-turning-condition)
                  {:input {:ultrasonic-active? true}})]
        (if (phase/phase-done? cmd :us-turning-condition)
          (phase/merge-cmds cmd
            {:state {:status :straightening
                     :sub-phases {:follow {:high-power? false}}}
             :input {:ultrasonic-active? false}})
          cmd))
      ; :turning
      ; (let [cmd (phase/merge-cmds
      ;             (phase/tick-subphase robot :turning)
      ;             (phase/tick-subphase robot :follow))]
      ;   (if (phase/phase-done? cmd :turning)
      ;     (phase/merge-cmds cmd
      ;       {:state {:status :straightening}})
      ;     cmd))
      :straightening
      (let [cmd (phase/merge-cmds
                  (phase/tick-subphase robot :until-straight)
                  (phase/tick-subphase robot :follow))]
        (if (phase/phase-done? cmd :until-straight)
          (phase/mark-done cmd)
          cmd)))))

(defphase up-to-ramp
  "Drives robot (pointed right) from home side of the table
  up to the ramp, then travelling a significant straight distance across it"
  :init
  {:status :find-turn}
  :sub-phases
  {:follow [biased-follow {:bias :left}]
   :turn-us-condition [tracking-prolonged-condition
                       {:min-duration 400
                        :pred (fn [{:keys [readings]}]
                                (<= 500 (:ultrasonic-2 readings) 800))}]
   :ramp-follow [basic-follow nil {:high-power? true}]
   ; :until-turn [until-turning {:turn-direction :left}]
   :until-straight [until-straight {:min-straight-duration 3000}]}
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :find-turn
      (let [cmd (phase/merge-cmds
                  (phase/tick-subphase robot :follow)
                  (phase/tick-subphase robot :turn-us-condition)
                  {:input {:ultrasonic-active? true}})]
        (if (phase/phase-done? robot :turn-us-condition)
          (phase/merge-cmds cmd
            {:state {:status :straightening
                     :sub-phases
                     {:ramp-follow
                      {:follow-intent
                       (get-in state [:sub-phases :follow :follow-intent])}}}
             :input {:ultrasonic-active? false}})
          cmd))
      :straightening
      (let [cmd (phase/merge-cmds
                  (phase/tick-subphase robot :until-straight)
                  (phase/tick-subphase robot :ramp-follow))]
        (if (phase/phase-done? cmd :until-straight)
          (phase/mark-done cmd)
          cmd)))))

(defphase exit-start-turn
  "Once at the start junction, turn 90° in preparation for
  line following"
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [{:keys [readings]
        {:keys [line-triggers]} :state}]
    (let [sturn 100
          sturnf 170
          [_ _ ntriggers-right ntriggers-far-right
           :as line-triggers] (update-line-triggers line-triggers readings)
          ;; done when right two sensors have encountered the line
          done? (and (pos? ntriggers-right) (pos? ntriggers-far-right))]
      (if done?
        (phase/mark-done {})
        {:state {:line-triggers line-triggers}
         :input (motor-input sturnf sturn)}))))

(defphase exit-start-find-junction
  "Moves the robot out of the start box until the line sensors
  have found the junction.
  At least three line sensors should find a distinct white region twice."
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [{:keys [readings]
        {:keys [line-triggers]} :state}]
    (let
      [sforward 200
       line-triggers (update-line-triggers line-triggers readings)
       ;; one line sensor may remain on the path and not be triggered twice
       found-junction?
       (let [non-ones (filterv #(not= % 1) line-triggers)]
         (and (<= 2 (count non-ones))
           (every? #(<= 2 %) non-ones)))
       done?
       (or (<= 340 (:ultrasonic-2 readings) 600)
         (and found-junction?
           (every? #(= % :black) (robot.state/get-line-sensors readings))))]
      (if done?
        (phase/mark-done {:input {:ultrasonic-active? false}})
        (let [speed sforward]
          {:state {:line-triggers line-triggers}
           :input (assoc (motor-input speed)
                    :ultrasonic-active? true)})))))

(defphase exit-start
  :init {:status :find}
  :sub-phases
  {:find [exit-start-find-junction]
   :excess [timed-forwards {:duration 200}]
   :turn [exit-start-turn]}
  :tick
  (fn [{:keys [state] :as robot}]
    (case (:status state)
      :find
      (let [cmd (phase/tick-subphase robot :find)]
        (if (phase/phase-done? cmd :find)
          (update cmd :state assoc :status :excess)
          cmd))
      :excess
      (let [cmd (phase/tick-subphase robot :excess)]
        (if (phase/phase-done? cmd :excess)
          (update cmd :state assoc :status :turn)
          cmd))
      :turn
      (let [cmd (phase/tick-subphase robot :turn)]
        (if (phase/phase-done? cmd :turn)
          (phase/mark-done cmd)
          cmd)))))

(let [initial-phases {:exit-start :up-to-ramp
                      :up-to-ramp :post-ramp-find-junction
                      :post-ramp-find-junction :post-ramp-to-centre-block
                      :post-ramp-to-centre-block :detect-block
                      :detect-block :tunnel-approach}
      common-phases {:tunnel-approach :through-tunnel
                     :decide-next-target :backup-from-box-turn
                     :post-tunnel-to-collect :collect-from-junction
                     
                     :up-to-home-entry :finalise-home}
      collect->box-phases {:through-tunnel :up-to-dropoff-box
                           :up-to-dropoff-box [:junction-approach-turn {:turn-direction :right}]
                           :junction-approach-turn :box-approach-edge
                           :box-approach-edge :backup-from-box
                           :backup-from-box :decide-next-target}
      dropoff->collect-phases {:backup-from-box-turn :tunnel-approach
                               :through-tunnel :post-tunnel-to-collect
                               :post-tunnel-to-collect :collect-from-junction
                               :collect-from-junction :detect-block
                               :detect-block :branch-to-main
                               :branch-to-main :tunnel-approach}]
  (defphase full-run
    :init
    (fn [{:keys [id pm]}]
      {:competition-start-time (System/currentTimeMillis)
       :go-home? nil
       :collection-target 2 ;; 1–3 from left to right
       :nest {:current-id (or id :exit-start)
              :next-phase-map (merge common-phases
                                (case pm
                                  1 collect->box-phases
                                  2 dropoff->collect-phases
                                  initial-phases))}})
    :tick
    (fn [{:keys [] :as robot}]
      (phase/tick-mapped-phase-group robot :nest
        (fn [_robot {{:keys [block-detected next-target]} :output
                     :as cmd}]
          (cond-> cmd
            block-detected
            (as-> cmd
              (update cmd :state
                (fn [state2]
                  (-> state2
                    (assoc :density (:density block-detected))
                    (update-in [:nest :next-phase-map] merge collect->box-phases)))))
        
            next-target
            (as-> cmd
              (let [{:keys [go-home? collection-target]} next-target
                    go-home? (or (nil? collection-target) go-home?)]
                (update cmd :state
                  (fn [state2]
                    (-> state2
                      (assoc :go-home? go-home?)
                      (assoc :collection-target collection-target)
                      (update-in [:nest :next-phase-map] merge
                        (if go-home?
                          {:backup-from-box-turn :up-to-home-entry}
                          dropoff->collect-phases)))))))))))))

(defn tick [{:keys [state global-state merged-state parent-phases] :as prev-robot}]
  (let [current-phase-id (:phase-id state)
        _ (when (nil? current-phase-id)
            (throw (ex-info "Phase is nil"
                     {:state (select-keys state [:phase-id])})))
        current-phase (phase/lookup-phase current-phase-id)
        cmd ((:tick-fn current-phase)
             (assoc prev-robot :merged-state (merge merged-state state)))
        cmd (phase/merge-cmds prev-robot cmd)
        {state2 :state} cmd
        next-phase-id (if (phase/phase-done? cmd)
                        ((:next-phase-map global-state {})
                         current-phase-id)
                        (:phase-id state2 current-phase-id))
        next-phase-id (or next-phase-id :stop)
        next-phase (phase/lookup-phase next-phase-id)]
    (cond
      (contains? parent-phases next-phase-id)
      (throw (ex-info "Circular phase reference"
               {:current-phase current-phase
                :next-phase next-phase
                :parent-phases parent-phases
                :next-phase-map (:next-phase-map global-state)}))
      ;; if transitioning to a new phase, run it immediately
      (not= current-phase-id next-phase-id)
      (do (println "Transition: " current-phase-id " -> " next-phase-id)
        (recur
          (assoc cmd :state
            (-> state2
              (assoc :parent-phases (conj parent-phases current-phase-id))
              (phase/initialise-phase-on-state next-phase)))))
      :else
      (update cmd :state dissoc :parent-phases))))

(defn tick! [*state readings input]
  (let [prev-state @*state
        {:keys [input state]}
        (tick {:state (:phase prev-state)
               :global-state (select-keys prev-state [:next-phase-map])
               :merged-state prev-state
               :readings readings
               :input input})]
    (reset! *state (assoc prev-state :phase state))
    input))