(ns idp.robot.brain.core
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain
     [phase :as phase :refer [defphase]]
     box
     ramp
     start
     tunnel
     detect
     travel
     [util :as bu]]))

(let [initial-phases {:exit-start :cross-from-start-by-ramp
                      :cross-from-start-by-ramp :post-ramp-to-centre-block
                      :post-ramp-to-centre-block :detect-block
                      :detect-block :up-to-tunnel-end}
      common-phases {:decide-next-target :backup-from-box-turn
                     :post-tunnel-to-collect :collect-from-junction
                     
                     :up-to-home-entry :finalise-home}
      collect->box-phases {:up-to-tunnel-end :up-to-dropoff-box
                           :up-to-dropoff-box [:junction-approach-turn {:turn-direction :right}]
                           :junction-approach-turn :box-approach-edge
                           :box-approach-edge :backup-from-box
                           :backup-from-box :decide-next-target}
      dropoff->collect-phases {:backup-from-box-turn :up-to-tunnel-end
                               :up-to-tunnel-end :post-tunnel-to-collect
                               :post-tunnel-to-collect :collect-from-junction
                               :collect-from-junction :detect-block
                               :detect-block :branch-to-main
                               :branch-to-main :up-to-tunnel-end}]
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

(defn tick!
  "Executes the current phase and updates the robot state"
  [*state readings input]
  (let [prev-state @*state
        phase-state (:phase prev-state)
        cmd
        (phase/tick-phase
          (phase/lookup-phase
            (enc/have some? (:phase-id phase-state)))
         {:state phase-state
          :global-state (select-keys prev-state [:next-phase-map])
          :merged-state (merge prev-state phase-state)
          :readings readings
          :readings-history (:readings-history prev-state)
          :input input})]
    (swap! *state assoc :phase
      (if (phase/phase-done? cmd)
        (phase/get-initial-state bu/stop)
        (phase/merge-states phase-state (:state cmd))))
    (merge input (:input cmd))))
