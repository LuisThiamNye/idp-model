(ns idp.robot.brain.travel
  (:require
    [taoensso.encore :as enc]
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
           [:left 1]  [:left 2]
           [:left 3]  [:left 4]
           [:right 1] [:right 2]
           [:right 3] [:right 4]
           :else prev-intent)
         :else prev-intent)))})

(defphase basic-follow
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
                            {:straight :straight
                             :left :right
                             :right :left})
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
       forward-speed max-fspeed
       done? (<= min-blackouts-duration blackout-duration)]
      (if done?
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
        (phase/mark-done {})
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

(defphase tracking-active-duration
  "Keeps track of how much time has passed while the robot is unpaused"
  :init {:duration 0}
  :tick
  (fn [{:keys [state readings]}]
    {:state {:duration (+ (:duration state)
                         (robot.state/get-active-dt readings))}}))

(defphase junction-turn-spin
  "Turn from one junction exit to another.
  At the end, line sensors should be centred on the line."
  :init (fn [{:keys [turn-direction]}]
          {:location :start-line ;; :start-line, :between, :end-line
           :turn-direction turn-direction})
  :tick
  (fn [{:keys [state readings]}]
    (let [forward-speed -30
          turn-speed 180
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

(defphase until-straight
  "Infers, based on motor input, when robot starts
  going approximately straight"
  :init (fn [params]
          {:min-straight-duration (:min-straight-duration params 1000)})
  :sub-phases
  {:duration [tracking-active-duration]
   :turn [tracking-motor-turn]}
  :tick
  (fn [{:keys [state] :as robot}]
    (let [;; after this turn amount, restart timer
          turn-amount-thres (long 6e4)
          duration-cmd (phase/tick-subphase robot :duration)
          duration (-> duration-cmd :state :sub-phases :duration :duration)
          turn-cmd (phase/tick-subphase robot :turn)
          turn-amount (-> turn-cmd :state :sub-phases :turn :turn-amount)
          cmd (phase/merge-cmds turn-cmd duration-cmd)
          reset? (<= turn-amount-thres (abs turn-amount))]
      (cond
        reset?
        (-> cmd
          (assoc-in [:state :sub-phases :duration :duration] 0)
          (assoc-in [:state :sub-phases :turn :turn-amount] 0))
        (<= (:min-straight-duration state) duration)
        (phase/mark-done cmd)
        :else
        cmd))))

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

(defphase until-turning
  "Infers, based on motor input, when robot starts turning"
  :init
  (fn [{:keys [turn-direction]}]
    {:min-turn-rate 200 ;; turn amount per active millisecond
     :turn-direction turn-direction})
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
        (phase/mark-done {})
        cmd))))

(defn reattempt-collection?
  "After dropping off block, decides whether to try collecting another
  block or to go back home"
  [{:keys [competition-start-time]}]
  (let [min-collect-duration (* 60 1000)
        competition-duration (* 5 60 1000) ;; 5 mins
        time-passed (- (System/currentTimeMillis) competition-start-time)
        time-remaining (- competition-duration time-passed)]
    (<= min-collect-duration time-remaining))
  
  ;; FIXME
  false
  )

(defphase box-approach-turn-spin
  "Assume aligned with path, so that we can spin 90° to point
  towards the box. Line sensors should be centred on the line."
  :init (fn [{:keys [turn-direction]}]
          (merge
            (phase/get-initial-state junction-turn-spin)
            {:turn-direction (enc/have #{:left :right} turn-direction)}))
  :tick
  (fn [{:keys [] :as robot}]
    ((:tick-fn junction-turn-spin) robot)))

(defphase box-approach-turn-excess
  "Moves robot forwards, from position where line sensors are at the junction
  path, to where the centre of rotation is approximately aligned with
  the path of a box"
  :init (fn [{:keys [turn-direction]}]
          {:status :pre-align
           :turn-direction turn-direction})
  :sub-phases
  (fn [{:keys [turn-direction]}]
    {:forward-amount [tracking-motor-forward]
     :follow [biased-follow {:bias (case turn-direction
                                     :right :left
                                     :left :right)}]})
  :tick
  (fn [{:keys [readings]
        {:keys [status turn-direction]} :state
        :as robot}]
    (let [min-fwd-amount (long 10e5)
          cmd (phase/tick-subphase robot :forward-amount)
          forward-amount (-> cmd :state :sub-phases :forward-amount :forward-amount)]
      (if (<= min-fwd-amount forward-amount)
        (phase/mark-done {})
        (phase/merge-cmds cmd
          (phase/tick-subphase robot :follow)
          (when (= :pre-align status)
            ;; initially turn away from box a bit to avoid following the path up to it
            (let [ls (get-combined-line-readings readings)
                  nwhites (count (filterv #(= :w %) ls))
                  aligned? (<= nwhites 1)]
              ; (prn aligned? nwhites)
              (if aligned?
                {:state {:status :follow}}
                {:input (motor-input 0 (cond-> 200 (= :right turn-direction) -))}))))))))

(defphase box-approach-turn
  :init {:status :excess}
  :sub-phases
  (fn [{:keys [turn-direction]}]
    (enc/have? #{:left :right} turn-direction)
    {:excess [box-approach-turn-excess {:turn-direction turn-direction}]
     :spin [box-approach-turn-spin {:turn-direction turn-direction}]})
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

(defphase up-to-box
  "Goes from somewhere on the home-side main line up to one of the
  three junctions of a line box.
  Performs biased line following to ensure junction detection."
  :init (fn [{:keys [box-number turn-direction]}]
          {:box-number (enc/have #{1 2 3} box-number)
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
                            [_ _ :b :b] false
                            [:b :b :w :b] false
                            :else true)
                          (match normalised-ls
                            [_ _ :w :w] true
                            [:w :w :w _] true
                            :else false))
          nfinds (cond-> (:nfinds state) (and (not on-junction?)
                                           (not= prev-on-junction? on-junction?)) inc)
          target-box (:box-number state)
          cmd (cond-> (phase/tick-subphase robot :follow)
                ;; avoid following box path when at junction
                on-junction?
                (phase/merge-cmds
                  {:input (motor-input 180 (cond-> 80 (= :right turn-direction) -))}))]
      (if (<= target-box nfinds)
        (phase/mark-done cmd)
        (phase/merge-cmds
          cmd
          {:state {:on-junction? on-junction?
                   :nfinds nfinds}})))))

(defphase up-to-dropoff-box
  "Goes from tunnel up to one of the three junctions of a line box.
  Performs right-biased line following to ensure junction detection."
  :init {}
  :sub-phases
  (fn [{:keys [density]}]
    {:find-box [up-to-box {:box-number (case density
                                         :high 3 ;; red box
                                         :low 1 ;; green box
                                         (throw (ex-info "No density!" {})))
                           :turn-direction :right}]})
  :tick
  (fn [{:keys [] :as robot}]
    (let [cmd (phase/tick-subphase robot :find-box)]
      (if (phase/phase-done? cmd :find-box)
        (phase/mark-done cmd)
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
      {:find-turn [up-to-box {:turn-direction turn-direction
                              :box-number 1}]
       :turn [box-approach-turn {:turn-direction turn-direction}]}))
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

(defphase backup-from-box
  "Reverse from dropping off block at box"
  :init {:status :blackout-retreating}
  :sub-phases
  {:straight [timed-forwards {:duration 1200
                              :speed -255}]
   :turn [junction-turn-spin]}
  :tick
  (fn [{:keys [state readings merged-state] :as robot}]
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
          (let [home-direction (case (:density merged-state)
                                 :high :right
                                 :low :left)
                block-direction :left
                [go-home? turn-direction]
                (if (reattempt-collection? merged-state)
                  [false block-direction]
                  [true home-direction])]
            (phase/merge-cmds cmd
              {:state {:status :turning
                       :sub-phases
                       {:turn {:turn-direction turn-direction}}}
               :output {:go-home? go-home?}}))
          cmd))
      :turning
      (let [cmd (phase/tick-subphase robot :turn)]
        (if (phase/phase-done? cmd :turn)
          (phase/mark-done cmd)
          cmd)))
    ))

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
  Drives robot in straight line at constant speed."
  :init {:nfinds 0}
  :tick
  (fn [{:keys [state readings]}]
    (let [min-finds 3 ;; require multiple finds for extra certainty
          speed 255
          combined-line-readings (get-combined-line-readings readings)
          found-line? (match combined-line-readings
                        [:w  _ :b :b] true
                        [ _ :w :b :b] true
                        [:b :w  _ :b] true
                        [:b  _ :w :b] true
                        [:b :b :w  _] true
                        [:b :b  _ :w] true
                        :else false)
          nfinds (cond-> (:nfinds state) found-line? inc)
          done? (<= min-finds nfinds)]
      (if done?
        (phase/mark-done {})
        {:state {:nfinds nfinds}
         :input {:motor-1 speed
                 :motor-2 speed}}))))

(defphase tunnel-approach
  "Does line following up to tunnel.
  Tunnel starts after observing a certain number of line sensor
  blackouts."
  :init (phase/get-initial-state follow-up-to-blackout)
  :tick #((:tick-fn follow-up-to-blackout) %))

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
          time-elapsed (+ (:time-elapsed state)
                         (robot.state/get-active-dt readings))]
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
         :output {:density density}})
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
  {:follow [biased-follow {:bias :left}]}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [combined-readings (get-combined-line-readings readings)
          cmd (phase/tick-subphase robot :follow)
          cmd (phase/merge-cmds {:state state} cmd)
          cmd
          (if (= combined-readings
                (:combined-line-readings state))
            cmd
            (match combined-readings
              [:w :w :w :w]
              (-> cmd
                (update-in [:state :rhs-lines-found] inc)
                (update-in [:state :lhs-lines-found] inc))
              [:b :w :w :w]
              (update-in cmd [:state :rhs-lines-found] inc)
              [:w :w :w :b]
              (update-in cmd [:state :lhs-lines-found] inc)
              :else
              cmd))
          state (:state cmd)
          done? (and (<= 1 (:rhs-lines-found state))
                  (or (<= 2 (:lhs-lines-found state))
                    (and (<= 1 (:lhs-lines-found state))
                      (:block-present? readings))))]
      (if done?
        (phase/mark-done {})
        (assoc-in cmd [:state :combined-line-readings]
          combined-readings)))))

(defphase post-ramp-turning
  :init {}
  :sub-phases
  {:turn [tracking-motor-turn]}
  :tick
  (fn [{:as robot}]
    (let [;; after this turn amount, we have turned enough
          turn-amount-thres (long 1e6)
          turn-cmd (phase/tick-subphase robot :turn)
          done? (<= (-> turn-cmd :state :sub-phases :turn :turn-amount)
                  (- turn-amount-thres))]
      (if done?
        (phase/mark-done {})
        turn-cmd))))

(defphase post-ramp-find-junction
  :init {:status :find-turn}
  :sub-phases
  {:until-turning [until-turning {:turn-direction :left}]
   :turning [post-ramp-turning]
   :follow [basic-follow nil {:high-power? true}]}
  :tick
  (fn [{:keys [state reinit?] :as robot}]
    (case (:status state)
      :find-turn
      (let [ut-cmd (phase/tick-subphase robot :until-turning)]
        (if (phase/phase-done? ut-cmd :until-turning)
          (recur (assoc robot
                   :reinit? true
                   :state
                   (-> state
                     (assoc :status :turning)
                     (assoc-in [:sub-phases :follow :high-power?] false))))
          (phase/merge-cmds ut-cmd
            (phase/tick-subphase robot :follow))))
      :turning
      (let [cmd (phase/tick-subphase robot :turning)]
        (if (phase/phase-done? cmd :turning)
          (phase/mark-done {})
          (phase/merge-cmds
            (when reinit? {:state state})
            cmd
            (phase/tick-subphase robot :follow)))))))

(defphase up-to-ramp
  "Drives robot (pointed right) from home side of the table
  up to the ramp, then travelling a significant straight distance across it"
  :init
  {:status :find-turn}
  :sub-phases
  {:follow [basic-follow]
   :turn [tracking-motor-turn]
   :until-straight [until-straight {:min-straight-duration 3000}]}
  :tick
  (fn [{:keys [state reinit?] :as robot}]
    (case (:status state)
      :find-turn
      (let [;; after this turn amount, we have turned enough
            turn-amount-thres (long 1e6)
            turn-cmd (phase/tick-subphase robot :turn)
            done? (<= (-> turn-cmd :state :sub-phases :turn :turn-amount)
                    (- turn-amount-thres))]
        (if done?
          (recur (assoc robot
                   :reinit? true
                   :state
                   (-> state
                     (assoc :status :straightening)
                     (assoc-in [:sub-phases :follow :high-power?] true))))
          (phase/merge-cmds turn-cmd
            (phase/tick-subphase robot :follow))))
      :straightening
      (let [straight-cmd (phase/tick-subphase robot :until-straight)]
        (if (phase/phase-done? straight-cmd :until-straight)
          (phase/mark-done {})
          (phase/merge-cmds
            (when reinit? {:state state})
            straight-cmd
            (phase/tick-subphase robot :follow)))))))

(defphase exit-start-turn
  "Once at the start junction, turn 90° in preparation for
  line following"
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [{:keys [state readings]}]
    (let [sturn 120
          sturnf 170
          {:keys [line-triggers]} state
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
  (fn [{:keys [state readings]}]
    (let
      [sforward 200
       {:keys [line-triggers] :as _state} state
       line-triggers (update-line-triggers line-triggers readings)
       ;; one line sensor may remain on the path and not be triggered twice
       found-junction?
       (let [non-ones (filterv #(not= % 1) line-triggers)]
         (and (<= 2 (count non-ones))
           (every? #(<= 2 %) non-ones)))
       done?
       (and found-junction?
         (every? #(= % :black) (robot.state/get-line-sensors readings)))]
      (if done?
        ;; FIXME do excess forwards movement instead
        (phase/mark-done
          {:readings {:line-switches [0 0 0 0]}})
        (let [speed sforward]
          {:state {:line-triggers line-triggers}
           :input (motor-input speed)})))))

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

(defphase full-run
  :init (fn [{:keys [id]}]
          {:nest {:current-id (or id :exit-start)
                  :next-phase-map
                  {:exit-start :up-to-ramp
                   :up-to-ramp :post-ramp-find-junction
                   :post-ramp-find-junction :post-ramp-to-centre-block
                   :post-ramp-to-centre-block :detect-block
                   :detect-block :tunnel-approach
                   :tunnel-approach :through-tunnel
                   :through-tunnel :up-to-dropoff-box
                   :up-to-dropoff-box [:box-approach-turn {:turn-direction :right}]
                   :box-approach-turn :box-approach-edge
                   :box-approach-edge :backup-from-box}}})
  :tick
  (fn [{:keys [] :as robot}]
    (let [{{:keys [density go-home?]} :output
           :as cmd} (phase/tick-mapped-phase-group robot :nest)]
      (cond-> cmd
        density
        (assoc-in [:state :density] density)
        (some? go-home?)
        (assoc-in [:state :go-home?] go-home?)
        (some? go-home?)
        (update-in [:state :nest :next-phase-map] merge
          (if go-home?
            {:backup-from-box :up-to-home-entry}
            {:backup-from-box :up-to-ramp}))))))

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