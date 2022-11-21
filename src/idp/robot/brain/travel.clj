(ns idp.robot.brain.travel
  (:require
    [idp.net.api :as api]
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
  :tick (fn [_ _] {:input (motor-input 0)}))

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

(defn get-combined-line-readings
  "Results of whether each line sensor saw white at least once
  since last response.
  Returns 4-tuple of :w or :b"
  [readings]
  (let [line-triggers (get-line-triggers readings)]
    (-> (mapv #(if (or (= :white %1) (pos? %2)) :w :b)
          (robot.state/get-line-sensors readings)
          line-triggers)
      rseq vec)))

(defphase basic-follow
  :init {:follow-intent [:straight]}
  :tick
  (fn [state readings]
    (let
      [combined-readings (get-combined-line-readings readings)
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
      {:state {:follow-intent intent}
       :input (motor-input forward-speed turn-speed)})))

(defphase biased-follow
  :init {:follow-intent [:straight]
         :bias :left}
  :tick
  (fn [state readings]
    (let
      [combined-readings (get-combined-line-readings readings)
       mirror-intent #(update % 0
                        {:straight :straight
                         :left :right
                         :right :left})
       
       ;; Conformed to left bias
       swap? (= :right (:bias state))
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
         :else prev-intent)
       intent (cond-> intent swap? mirror-intent)
       ;; Reverted to actual bias
       
       [forward-speed turn-speed]
       (match intent
         [:straight] [255 0]
         [left-or-right level]
         (let [[forward turn]
               (case (int level)
                 1 [230 30]
                 2 [80 150]
                 3 [0 250])]
           [forward
            (cond-> turn (= :left left-or-right) -)]))]
      {:state {:follow-intent intent}
       :input (motor-input forward-speed turn-speed)})))

(defphase follow-up-to-blackout
  "Does line following to tightly follow the line until it suddenly ends.
  Requires a certain number of 'blackout' readings to end."
  :init {:follow-intent [:straight]
         :nblackouts 0}
  :tick
  (fn [state readings]
    (let
      [nblackouts-thres 2 ;; determines when phase is done
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
         :else prev-intent)
       nblackouts (:nblackouts state)
       max-fspeed (if (pos? nblackouts) blackout-speed 255)
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
       done? (<= nblackouts-thres nblackouts)]
      (if done?
        {:state {:phase-id nil}}
        {:state {:follow-intent intent
                 :nblackouts
                 (if (= [:b :b :b :b] combined-readings)
                   (inc nblackouts)
                   0)}
         :input (motor-input forward-speed turn-speed)}))))

(defphase straight-until-line
  "Drives straight until a number of non-blackouts are observed"
  :init {:speed 200}
  :tick
  (fn [state readings]
    ))

#_(defn tick-junction-turn-tracking
  "Turn until the line sensor on the desired side has crossed
  the target line"
  [{:keys [turn-direction] :as state} readings]
  (let [forward-speeds [100 60 0]
        turn-speeds [0 250 250]
        combined-line-readings (get-combined-line-readings readings)
        ;; Think as though turning right
        left? (= :left turn-direction)
        prev-turn (:turn-level state 1)
        turn-level
        (match (cond-> combined-line-readings
                 left? (-> rseq vec))
          [_ _ _ :w] 1
          [_ _ :w :b] nil
          [:b :b :b :b]
          (if (= 1 prev-turn)
            2
            prev-turn)
          :else prev-turn)
        done? (nil? turn-level)]
    (if done?
      {:state {:phase-id nil}}
      (let [turn-speed (cond-> (nth turn-speeds turn-level) left? -)
            forward-speed (nth forward-speeds turn-level)]
        {:input {:motor-1 (+ forward-speed turn-speed)
                 :motor-2 (- forward-speed turn-speed)}}))))

(defphase box-approach-edge
  "Go up to edge of box"
  :init (phase/initialise-phase-on-state
          {:turn-direction :right}
          follow-up-to-blackout)
  :tick
  (fn [state readings]
    (let [#_
          (match (get-combined-line-readings readings)
            [])]
      ((:tick-fn follow-up-to-blackout) state readings))))

(defphase timed-forwards
  "Move straight forwards for fixed amount of time at a given speed"
  :init {:duration 0
         :speed 200
         :time-elapsed 0}
  :tick
  (fn [{:keys [speed] :as state} readings]
    (let [time-elapsed (+ (:time-elapsed state) (:dt readings))
          done? (<= (:duration state) time-elapsed)]
      (if done?
        {:state {:phase-id nil}}
        {:state {:time-elapsed time-elapsed}
         :input (motor-input speed)}))))

(defphase junction-turn-spin
  "Turn from one junction exit to another.
  At the end, line sensors should be centred on the line."
  :init {:location :start-line ;; :start-line, :between, :end-line
         :turn-direction :left}
  :tick
  (fn [state readings]
    (let [turn-speed 250
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
             [_  _ :w  _]
             [(= :end-line location) :between]
             [:b _ _ :w]
             [false :end-line]
             :else
             [false location]))]
      (if done?
        {:state {:phase-id nil}}
        {:state {:location location}
         :input (motor-input 0 turn-speed)}))))

(defphase backup-from-box
  "Reverse from dropping off block at box"
  :init (merge (phase/get-initial-state timed-forwards)
          {:turn-direction :left})
  :tick
  (fn [state readings]
    ((:tick-fn timed-forwards)
     (assoc state
       :duration 1500
       :speed -200)
     readings)))

(defphase box-approach-turn-spin
  "Assume aligned with path, so that we can spin 90° to point
  towards the box. Line sensors should be centred on the line."
  :init {:found-line? false}
  :tick
  (fn [state readings]
    (let [turn-speed 250
          combined-line-readings (get-combined-line-readings readings)
          [done? found-line?]
          (match combined-line-readings
            [_ :w  _  _] [(and (:found-line? state) true) false]
            [:b :b _ :w] [false true]
            :else [false (:found-line? state)])]
      (if done?
        {:state {:phase-id nil}}
        {:state {:found-line? found-line?}
         :input (motor-input 0 turn-speed)}))))

(defphase box-approach-turn
  "Turns robot from main loop to be approximately aligned with
  the path of a box"
  :init (merge (phase/get-initial-state timed-forwards)
          {:time-elapsed 0})
  :tick
  (fn [state readings]
    (let [straight-duration 2000
          time-elapsed (+ (:time-elapsed state) (:dt readings))
          done? (<= straight-duration time-elapsed)]
      (if done?
        {:state {:phase-id nil}}
        (assoc-in ((:tick-fn basic-follow) state readings)
          [:state :time-elapsed] time-elapsed)))))

#_(defphase box-approach-turn
  "Turn off main loop onto box path.
  Starts by getting ahead of the line, then spins on the spot
  until line is between the sensors."
  :init {:turn-direction :right
         :spin? false}
  :tick
  (fn [state readings]
    (let [forward-speeds [nil 200 150 100 50]
          turn-speeds [nil 0 80 80 80]
          spin-speed 200
          combined-line-readings (get-combined-line-readings readings)
          turn-level
          (when-not (:spin? state)
            (match combined-line-readings
              [ _  _  _ :w] 1
              [ _  _ :w :b] 2
              [ _ :w :b :b] 3
              [:w :b :b :b] 4
              :else ;; blackout
              nil))
          spin? (nil? turn-level)
          done? (when spin?
                  (match combined-line-readings
                    [_ :w _ _] true
                    :else false))]
      (if done?
        {:state {:phase-id nil}}
        {:state {:spin? spin?}
         :input
         (if spin?
           {:motor-1 spin-speed
            :motor-2 (- spin-speed)}
           (let [forward-speed (nth forward-speeds turn-level)
                 turn-speed (nth turn-speeds turn-level)]
             {:motor-1 (+ forward-speed turn-speed)
              :motor-2 (- forward-speed turn-speed)}))}))))

(defphase up-to-box
  "Goes from tunnel up to one of the three junctions of a line box.
  Performs right-biased line following to ensure junction detection."
  :init {:density :low
         :nfinds 0
         :bias :right}
  :tick
  (fn [state readings]
    (let [combined-line-readings (get-combined-line-readings readings)
          found-junction? (when (not= combined-line-readings
                                  (:combined-line-readings state))
                            (match combined-line-readings
                              [ _ :w :w :w] true
                              :else false))
          nfinds (cond-> (:nfinds state) found-junction? inc)
          target-box (case (:density state)
                       :high 1 ;; red box
                       :low 3 ;; green box
                       (throw (ex-info "No density!" {})))
          done? (<= target-box nfinds)]
      (if done?
        {:state {:phase-id nil}}
        (merge-with merge
          {:state {:combined-line-readings combined-line-readings
                   :nfinds nfinds}}
          ((:tick-fn biased-follow) state readings))))))

(defphase through-tunnel
  "Waits until refinding the line.
  Drives robot in straight line at constant speed."
  :init {:nfinds 0}
  :tick
  (fn [state readings]
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
        {:state {:phase-id nil}}
        {:state {:nfinds nfinds}
         :input {:motor-1 speed
                 :motor-2 speed}}))))

(defphase tunnel-approach
  "Does line following up to tunnel.
  Tunnel starts after observing a certain number of line sensor
  blackouts."
  :init (phase/get-initial-state follow-up-to-blackout)
  :tick #((:tick-fn follow-up-to-blackout) %1 %2))

(defphase signal-block-density
  :init {:density :high
         :time-elapsed 0}
  :tick
  (fn [state readings]
    (let [signal-duration 5000
          time-elapsed (+ (:time-elapsed state) (:dt readings))
          done? (<= signal-duration time-elapsed)]
      (if done?
        {:state {:phase-id nil}
         :input {:signal-block-density nil}}
        {:state {:time-elapsed time-elapsed}
         :input {:signal-block-density (:density state)
                 :motor-1 0 :motor-2 0}}))))

(defphase tick-position-grabber
  :init {:started? false
         :grabber-position :closed}
  :tick
  (fn [state readings]
    (let [input {:grabber-position (:grabber-position state)}]
      (if (:started? state)
        (let [done? (not (:grabber-moving? readings))]
          (if done?
            {:state {:phase-id nil}}
            {:input input}))
        {:state {:started? true}
         :input input}))))

(defphase detect-block
  "Closes grabber, determines density, and signals the density.
  The grabber remains closed."
  :init (phase/initialise-phase-on-state
          {:sub-status :closing}
          tick-position-grabber)
  :tick
  (fn [state readings]
    (case (:sub-status state)
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
  :init {:follow-intent [:straight]
         ; :line-triggers [0 0 0 0]
         :rhs-lines-found 0
         :lhs-lines-found 0
         :bias :left}
  :tick
  (fn [state readings]
    (let [combined-readings (get-combined-line-readings readings)
          follow-cmd ((:tick-fn biased-follow) state readings)
          cmd (merge-with merge {:state state} follow-cmd)
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
          found-centre-collection-junction?
          (and (<= 1 (:rhs-lines-found state))
            (<= 2 (:lhs-lines-found state)))
          done? (or found-centre-collection-junction?
                  (and found-first-collection-junction?
                    (:block-present? readings)))]
      (if done?
        {:state {:phase-id nil}}
        (assoc-in cmd [:state :combined-line-readings]
          combined-readings)))))

(defphase exit-start-turn
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [state readings]
    (let [sturn 120
          sturnf 170
          [_ _ ls-right ls-far-right]
          (robot.state/get-white-line-sensors readings)
          {:keys [line-triggers]} state
          line-triggers (update-line-triggers line-triggers readings)
          ;; done when right two sensors have encountered the line
          done? (every? pos? (subvec line-triggers 0 2))]
      (if done?
        {:state {:phase-id nil}}
        {:state (cond-> {:line-triggers line-triggers}
                  (not (or ls-far-right ls-right))
                  (assoc :over-horiz? false))
         :input {:motor-1 (+ sturnf sturn)
                 :motor-2 (- sturnf sturn)}}))))

(defphase exit-start
  :init {:line-triggers [0 0 0 0]}
  :tick
  (fn [state readings]
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
        {:state {:phase-id nil}
         :readings {:line-switches [0 0 0 0]}}
        (let [speed sforward]
          {:state {:line-triggers line-triggers}
           :input {:motor-1 speed
                   :motor-2 speed}})))))

(defn tick [{:keys [state readings] :as prev-cmd}]
  (let [current-phase-id (:phase-id state)
        _ (when (nil? current-phase-id)
            (throw (ex-info "Phase is nil"
                     {:state (select-keys state
                               [:parent-phases :phase-id])})))
        current-phase (phase/lookup-phase current-phase-id)
        tick-fn (:tick-fn current-phase)
        cmd (tick-fn state readings)
        cmd (merge-with merge prev-cmd cmd)
        {state2 :state} cmd
        next-phase-id (if (nil? (:phase-id state2))
                        ((:next-phase-map state2 {})
                         current-phase-id)
                        current-phase-id)
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

(defn tick! [*state readings]
  (let [prev-state @*state
        {:keys [input state]} (tick {:state prev-state
                                     :readings readings})]
    ; (compare-and-set! *state prev-state state)
    (reset! *state state)
    input))