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
  :tick (fn [_] {:input (motor-input 0)}))

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
             [ _ :w :w  _] [:straight]
             [:b :w :b :b] [:left 2]
             [:w  _ :b :b] [:left 3]
             [:b :b :w :b] [:straight]
             [ _  _ :w :w] [:straight]
             [ _ :b :b :w] [:right 2]
             [:b :b :b :b]
             (match prev-intent
               [:right 2] [:right 3]
               [:right 1] [:right 3]
               :else prev-intent)
             :else prev-intent)]
       (cond-> intent swap? mirror-intent)))})

(defphase biased-follow
  :init {:follow-intent [:continue]
         :bias :left
         :high-power? false}
  :tick
  (fn [{:keys [state] :as robot}]
    (tick-common-follow robot
      (assoc basic-follow-strategy :level-speeds
        (if (:high-power? state)
          [[255 0] [255 30] [80 255] [0 255]]
          [[255 0] [255 30] [80 150] [0 200]])))))

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
  (fn [{:keys [state readings]}]
    (let
      [min-blackouts-duration 400 ;; determines when phase is done (ms)
       blackout-speed 50 ;; after first blackout, go slower
       combined-readings (get-combined-line-readings readings)
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
         :else prev-intent)]
      (if (= [:continue] intent)
        {}
        (let [{:keys [blackout-duration]} state
              max-fspeed (if (pos? blackout-duration) blackout-speed 255)
              [forward-speed turn-speed]
              (match intent
                [:straight] [max-fspeed 0]
                [left-or-right level]
                (let [[forward turn]
                      (case (int level)
                        1 [(* max-fspeed 0.9) 30]
                        2 [20 200]
                        3 [0 250])]
                  [forward
                   (cond-> turn (= :left left-or-right) -)]))
              done? (<= min-blackouts-duration blackout-duration)]
          (if done?
            (phase/mark-done {})
            {:state {:follow-intent intent
                     :blackout-duration
                     (if (= [:b :b :b :b] combined-readings)
                       (+ blackout-duration (robot.state/get-active-dt readings))
                       0)}
             :input (motor-input forward-speed turn-speed)}))))))

(defphase timed-forwards
  "Move straight forwards (or backwards) for fixed amount of time at a given speed"
  :init (fn [params]
          {:duration (:duration params)
           :speed 200
           :turn-speed 0
           :time-elapsed 0})
  :tick
  (fn [{{:keys [speed] :as state} :state
        :keys [readings]}]
    (let [time-elapsed (+ (:time-elapsed state)
                         (robot.state/get-active-dt readings))
          done? (<= (:duration state) time-elapsed)]
      (if done?
        (phase/mark-done {})
        {:state {:time-elapsed time-elapsed}
         :input (motor-input speed (:turn-speed state))}))))

(defphase junction-turn-spin
  "Turn from one junction exit to another.
  At the end, line sensors should be centred on the line."
  :init {:location :start-line ;; :start-line, :between, :end-line
         :turn-direction :left}
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
  :init
  (fn [prev-state]
    {:sub-status :straight
     :sub-states
     {:biased-follow (assoc (phase/get-initial-state biased-follow)
                       :bias (if (= :high (:density prev-state))
                               :left
                               :right))
      :timed-forwards (assoc (phase/get-initial-state timed-forwards)
                        :duration 1700)
      :spin (assoc (phase/get-initial-state junction-turn-spin)
              :turn-direction (if (= :high (:density prev-state))
                                :left
                                :right))}})
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :straight
      (let [cmd (phase/tick-nested biased-follow :biased-follow
                  state readings)
            junction-found?
            (match (get-combined-line-readings readings)
              [_ :w :w :w] true
              [:w :w :w _] true
              :else false)]
        (if junction-found?
          (update cmd :state assoc
            :sub-status :excess)
          cmd))
      :excess
      (let [cmd (phase/tick-nested timed-forwards :timed-forwards state readings)]
        (if (phase/phase-done? cmd :timed-forwards)
          (update cmd :state assoc
            :sub-status :turning)
          cmd))
      :turning
      (let [cmd (phase/tick-nested junction-turn-spin :spin
                  state readings)]
        (if (phase/phase-done? cmd :spin)
          (phase/mark-done cmd)
          cmd)))))

(defphase backup-from-box
  "Reverse from dropping off block at box"
  :init (merge (phase/get-initial-state timed-forwards)
          {:sub-status :retreating})
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :retreating
      (let [cmd ((:tick-fn timed-forwards)
                 (assoc state
                   :duration 1500
                   :speed -200)
                 readings)]
        (if (phase/phase-done? cmd)
          (let [cmd (update cmd :state merge
                      {:phase-id (:phase-id state)
                       :sub-status :turning}
                      (phase/get-initial-state junction-turn-spin))
                home-direction (if (= :high (:density state))
                                 :right :left)
                block-direction :left]
            (if (reattempt-collection? state)
              (update cmd :state
                (fn [state2]
                  (-> state2
                    (assoc :turn-direction block-direction)
                    (assoc :go-home? false))))
              (update cmd :state
                (fn [state2]
                  (-> state2
                    (assoc :turn-direction home-direction)
                    (assoc :go-home? true))))))
          cmd))
      :turning
      (let [cmd ((:tick-fn junction-turn-spin) state readings)]
        (if (phase/phase-done? cmd)
          (update cmd :state assoc :phase-id
            ((:next-phase-map state {})
             [:backup-from-box
              {:go-home? (:go-home? state)}]))
          cmd)))
    ))

(defphase box-approach-edge
  "Go up to edge of box"
  :init (merge
          (phase/get-initial-state straight-up-to-blackout)
          {:turn-direction :right
           :sub-status :up-to-line})
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :up-to-line
      (let [cmd (assoc-in ((:tick-fn straight-up-to-blackout) state readings)
                  [:input :grabber-position] :open)]
        (if (phase/phase-done? cmd)
          (update cmd :state merge
            (phase/get-initial-state timed-forwards)
            {:phase-id (:phase-id state)
             :sub-status :excess})
          cmd))
      :excess
      ((:tick-fn timed-forwards)
       (assoc state :duration 300)
       readings))))

(defphase box-approach-turn-spin
  "Assume aligned with path, so that we can spin 90° to point
  towards the box. Line sensors should be centred on the line."
  :init (merge
          (phase/get-initial-state junction-turn-spin)
          {:turn-direction :right})
  :tick
  (fn [{:keys [state readings]}]
    ((:tick-fn junction-turn-spin) state readings)))

(defphase box-approach-turn
  "Turns robot from main loop to be approximately aligned with
  the path of a box"
  :init (merge (phase/get-initial-state timed-forwards)
          {:time-elapsed 0})
  :tick
  (fn [{:keys [state readings]}]
    (let [straight-duration 1700
          time-elapsed (+ (:time-elapsed state)
                         (robot.state/get-active-dt readings))
          done? (<= straight-duration time-elapsed)]
      (if done?
        (phase/mark-done {})
        (assoc-in ((:tick-fn basic-follow) state readings)
          [:state :time-elapsed] time-elapsed)))))

(defphase up-to-box
  "Goes from tunnel up to one of the three junctions of a line box.
  Performs right-biased line following to ensure junction detection."
  :init {:density :low
         :nfinds 0
         :bias :right}
  :tick
  (fn [{:keys [state readings]}]
    (let [combined-line-readings (get-combined-line-readings readings)
          found-junction? (when (not= combined-line-readings
                                  (:combined-line-readings state))
                            (match combined-line-readings
                              [ _ :w :w :w] true
                              :else false))
          nfinds (cond-> (:nfinds state) found-junction? inc)
          target-box (case (:density state)
                       :high 3 ;; red box
                       :low 1 ;; green box
                       (throw (ex-info "No density!" {})))
          done? (<= target-box nfinds)]
      (if done?
        (phase/mark-done {})
        (phase/merge-cmds
          {:state {:combined-line-readings combined-line-readings
                   :nfinds nfinds}}
          ((:tick-fn biased-follow) state readings))))))

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
  :init {:nturns 0
         :sub-states
         {:turn1 (assoc (phase/get-initial-state timed-forwards)
                   :duration 2000
                   :speed 0
                   :turn-speed 200)
          :turn2 (assoc (phase/get-initial-state junction-turn-spin)
                   :turn-direction :right)}}
  :tick
  (fn [{:keys [state readings]}]
    (let [nturns (:nturns state)
          first? (= 0 nturns)
          ss (if first? :turn1 :turn2)
          cmd (if first?
                (phase/tick-nested timed-forwards :turn1 state readings)
                (phase/tick-nested junction-turn-spin :turn2 state readings))]
      (if (phase/phase-done? cmd ss)
        (if first?
          (update cmd :state assoc :nturns (inc nturns))
          (phase/mark-done cmd))
        cmd))))

(defphase signal-block-density
  :init (fn [prev]
          {:density (:density prev :high)
           :time-elapsed 0})
  :tick
  (fn [{:keys [state readings]}]
    (let [signal-duration 5000
          time-elapsed (+ (:time-elapsed state)
                         (robot.state/get-active-dt readings))
          done? (<= signal-duration time-elapsed)]
      (if done?
        (phase/mark-done
          {:input {:signal-block-density nil}})
        {:state {:time-elapsed time-elapsed}
         :input {:signal-block-density (:density state)
                 :motor-1 0 :motor-2 0}}))))

(defphase tick-position-grabber
  :init {:started? false
         :grabber-position :closed}
  :tick
  (fn [{:keys [state readings]}]
    (let [input {:grabber-position (:grabber-position state)}]
      (if (:started? state)
        (let [done? (not (:grabber-moving? readings))]
          (if done?
            (phase/mark-done {})
            {:input input}))
        {:state {:started? true}
         :input input}))))

(defphase stationary-open-grabber
  :init (assoc (phase/get-initial-state tick-position-grabber)
          :grabber-position :open)
  :tick (fn [{:keys [state readings]}]
          ((:tick-fn tick-position-grabber) state readings)))

(defphase detect-block
  "Closes grabber, determines density, and signals the density.
  The grabber remains closed."
  :init (merge
          {:sub-status :pushing
           :sub-states {:pushing (assoc (phase/get-initial-state timed-forwards)
                                   :duration 500)}}
          (phase/get-initial-state tick-position-grabber))
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :pushing
      (let [cmd (phase/tick-nested timed-forwards :pushing state readings)]
        (if (phase/phase-done? cmd :pushing)
          {:state {:sub-status :closing}
           :input (motor-input 0)}
          cmd))
      :closing
      (let [r ((:tick-fn tick-position-grabber)
               (assoc state :grabber-position :closed)
               readings)]
        (if (phase/phase-done? r)
          {:state {:sub-status :detect}}
          r))
      :detect
      (let [density (:block-density readings)]
        {:state (merge (phase/get-initial-state signal-block-density)
                  {:density density
                   :sub-status :done-detect})})
      :done-detect
      ((:tick-fn signal-block-density) state readings))))

(defphase start-to-centre-block
  "Does left-biased line following up to central collection point.
  Conditions for stop:
  - Find the first collection junction (left turn, three line sensors)
  - Find the centre junction (three line sensors trigger on each side)"
  :init
  (fn [prev]
    {:rhs-lines-found 0
     :lhs-lines-found 0
     :sub-states {:biased-follow (assoc (phase/get-initial-state biased-follow)
                                   :high-power? false
                                   :bias (:bias prev))}})
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [combined-readings (get-combined-line-readings readings)
          cmd (phase/tick-nested biased-follow
                :biased-follow robot)
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
          found-first-collection-junction?
          (<= 1 (:lhs-lines-found state))
          ; found-centre-collection-junction?
          ; (and (<= 1 (:rhs-lines-found state))
          ;   (<= 2 (:lhs-lines-found state)))
          done? (or ;found-centre-collection-junction?
                  (and found-first-collection-junction?
                    (:block-present? readings)))]
      (if done?
        (phase/mark-done {})
        (assoc-in cmd [:state :combined-line-readings]
          combined-readings)))))

(defphase start-to-centre-block-tunnel
  :init
  {:sub-status :tunnel-approach
   :sub-states {:tunnel-approach (phase/get-initial-state tunnel-approach)
                :tunnel (phase/get-initial-state through-tunnel)
                :to-centre (phase/get-initial-state start-to-centre-block
                             {:bias :right})}}
  :tick
  (fn [{:keys [state readings]}]
    (case (:sub-status state)
      :tunnel-approach
      (let [cmd (phase/tick-nested tunnel-approach :tunnel-approach state readings)]
        (if (phase/phase-done? cmd :tunnel-approach)
          (update cmd :state assoc :sub-status :tunnel)
          cmd))
      :tunnel
      (let [cmd (phase/tick-nested through-tunnel :tunnel state readings)]
        (if (phase/phase-done? cmd :tunnel)
          (update cmd :state assoc :sub-status :to-centre)
          cmd))
      :to-centre
      (let [cmd (phase/tick-nested start-to-centre-block :to-centre state readings)]
        (if (phase/phase-done? cmd :to-centre)
          (update cmd :state assoc :phase-id
            ((:next-phase-map state {}) [(:phase-id state)]))
          cmd)))))

(defn calc-motor-turn-amount
  "Estimates, based on motor input, the amount by
  which the robot has turned clockwise since previous response"
  [{:keys [readings input]}]
  (let [turn-speed (- (:motor-1 input) (:motor-2 input))]
    (* turn-speed (robot.state/get-active-dt readings))))

(defphase tracking-motor-turn
  "Estimates, based on motor input, the amount by
  which the robot has turned clockwise"
  :init {:turn-amount 0}
  :tick
  (fn [{:keys [state] :as robot}]
    {:state {:turn-amount (+ (:turn-amount state)
                            (calc-motor-turn-amount robot))}}))

(defphase tracking-active-duration
  "Keeps track of how much time has passed while the robot is unpaused"
  :init {:duration 0}
  :tick
  (fn [{:keys [state readings]}]
    {:state {:duration (+ (:duration state)
                         (robot.state/get-active-dt readings))}}))

(defphase until-straight
  "Infers, based on motor input, when robot starts
  going approximately straight"
  :init {:min-straight-duration 1000
         :sub-states
         {:duration (phase/get-initial-state tracking-active-duration)
          :turn (phase/get-initial-state tracking-motor-turn)}}
  :tick
  (fn [{:keys [state] :as robot}]
    (let [;; after this turn amount, restart timer
          turn-amount-thres (long 6e4)
          duration-cmd (phase/tick-nested tracking-active-duration
                         :duration robot)
          duration (-> duration-cmd :state :sub-states :duration :duration)
          turn-cmd (phase/tick-nested tracking-motor-turn :turn robot)
          turn-amount (-> turn-cmd :state :sub-states :turn :turn-amount)
          cmd (phase/merge-cmds turn-cmd duration-cmd)
          reset? (<= turn-amount-thres (abs turn-amount))]
      (cond
        reset?
        (-> cmd
          (assoc-in [:state :sub-states :duration :duration] 0)
          (assoc-in [:state :sub-states :turn :turn-amount] 0))
        (<= (:min-straight-duration state) duration)
        (phase/mark-done cmd)
        :else
        cmd))))

(defphase tracking-motor-turn-rate
  "Calculates the turn rate over a fixed duration
  based on motor input"
  :init {:target-duration 700 ;; ms
         
         :history (enc/queue)
         :duration 0 ;; ms
         :turn-amount 0
         :turn-rate 0 ;; turn amount per active millisecond
         :sub-states
         {:turn (phase/get-initial-state tracking-motor-turn)}}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [turn-cmd (phase/tick-nested tracking-motor-turn :turn robot)
          Δturn-amount (-> turn-cmd :state :sub-states :turn :turn-amount)
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
     :turn-direction turn-direction
     :sub-states
     {:turn (assoc (phase/get-initial-state tracking-motor-turn-rate)
              :target-duration 700)}})
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let [cmd (phase/tick-nested tracking-motor-turn-rate :turn robot)
          {:keys [turn-rate duration target-duration]}
          (-> cmd :state :sub-states :turn)
          desired-direction? (case (:turn-direction state)
                               :left (neg? turn-rate)
                               :right (pos? turn-rate)
                               :either true)]
      (if (and (<= target-duration duration)
            desired-direction?
            (<= (:min-turn-rate state) (abs turn-rate)))
        (phase/mark-done {})
        cmd))))

(defphase post-ramp-turning
  :init {:sub-states
         {:turn (phase/get-initial-state tracking-motor-turn)}}
  :tick
  (fn [{:as robot}]
    (let [;; after this turn amount, we have turned enough
          turn-amount-thres (long 1e6)
          turn-cmd (phase/tick-nested tracking-motor-turn :turn robot)
          done? (<= (-> turn-cmd :state :sub-states :turn :turn-amount)
                  (- turn-amount-thres))]
      (if done?
        (phase/mark-done {})
        turn-cmd))))

(defphase post-ramp-find-junction
  :init {:sub-status :find-turn
         :sub-states
         {:until-turning (phase/get-initial-state until-turning
                           {:turn-direction :left})
          :turning (phase/get-initial-state post-ramp-turning)
          :follow (assoc (phase/get-initial-state basic-follow)
                    :high-power? true)}}
  :tick
  (fn [{:keys [state reinit?] :as robot}]
    (case (:sub-status state)
      :find-turn
      (let [ut-cmd (phase/tick-nested until-turning :until-turning robot)]
        (if (phase/phase-done? ut-cmd :until-turning)
          (recur (assoc robot
                   :reinit? true
                   :state
                   (-> state
                     (assoc :sub-status :turning)
                     (assoc-in [:sub-states :follow :high-power?] false))))
          (phase/merge-cmds ut-cmd
            (phase/tick-nested basic-follow :follow robot))))
      :turning
      (let [cmd (phase/tick-nested post-ramp-turning :turning robot)]
        (if (phase/phase-done? cmd :turning)
          (phase/mark-done {})
          (phase/merge-cmds
            (when reinit? {:state state})
            cmd
            (phase/tick-nested basic-follow :follow robot)))))))

(defphase up-to-ramp
  "Drives robot (pointed right) from home side of the table
  up to the ramp, then travelling a significant straight distance across it"
  :init
  {:sub-status :find-turn
   :sub-states
   {:follow (phase/get-initial-state basic-follow)
    :turn (phase/get-initial-state tracking-motor-turn)
    :until-straight (assoc (phase/get-initial-state until-straight)
                      :min-straight-duration 3000)}}
  :tick
  (fn [{:keys [state reinit?] :as robot}]
    (case (:sub-status state)
      :find-turn
      (let [;; after this turn amount, we have turned enough
            turn-amount-thres (long 1e6)
            turn-cmd (phase/tick-nested tracking-motor-turn :turn robot)
            done? (<= (-> turn-cmd :state :sub-states :turn :turn-amount)
                    (- turn-amount-thres))]
        (if done?
          (recur (assoc robot
                   :reinit? true
                   :state
                   (-> state
                     (assoc :sub-status :straightening)
                     (assoc-in [:sub-states :follow :high-power?] true))))
          (phase/merge-cmds turn-cmd
            (phase/tick-nested basic-follow :follow robot))))
      :straightening
      (let [straight-cmd (phase/tick-nested until-straight :until-straight robot)]
        (if (phase/phase-done? straight-cmd :until-straight)
          (phase/mark-done {})
          (phase/merge-cmds
            (when reinit? {:state state})
            straight-cmd
            (phase/tick-nested basic-follow :follow robot)))))))

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
  :init {:line-triggers [0 0 0 0]
         :competition-start-time (System/currentTimeMillis)}
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

(phase/get-initial-state exit-start)

(defn tick [{:keys [state] :as prev-robot}]
  (let [current-phase-id (:phase-id state)
        _ (when (nil? current-phase-id)
            (throw (ex-info "Phase is nil"
                     {:state (select-keys state
                               [:parent-phases :phase-id])})))
        current-phase (phase/lookup-phase current-phase-id)
        tick-fn (:tick-fn current-phase)
        cmd (tick-fn prev-robot)
        cmd (phase/merge-cmds prev-robot cmd)
        {state2 :state} cmd
        next-phase-id (if (phase/phase-done? cmd)
                        ((:next-phase-map state2 {})
                         current-phase-id)
                        (:phase-id state2 current-phase-id))
        next-phase-id (or next-phase-id :stop)
        next-phase (phase/lookup-phase next-phase-id)
        parent-phases (:parent-phases state #{})]
    (cond
      (contains? parent-phases next-phase-id)
      (throw (ex-info "Circular phase reference"
               {:current-phase current-phase
                :next-phase next-phase
                :parent-phases parent-phases
                :next-phase-map (:next-phase-map state2)}))
      ;; if transitioning to a new phase, run it immediately
      (not= current-phase-id next-phase-id)
      (recur
        (assoc cmd :state
          (-> state2
            (assoc :parent-phases (conj parent-phases current-phase-id))
            (phase/initialise-phase-on-state next-phase))))
      :else
      (update cmd :state dissoc :parent-phases))))

(defn tick! [*state readings input]
  (let [prev-state @*state
        {:keys [input state]} (tick {:state prev-state
                                     :readings readings
                                     :input input})]
    (reset! *state state)
    input))

;; TODO transfer follow-intent for phase transitions