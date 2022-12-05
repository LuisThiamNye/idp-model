(ns idp.robot.brain.phase
  "Infrastructure for the state machine."
  (:require
    [taoensso.encore :as enc]))

(def *phase-registry
  "Maps keyword to a phase definition var"
  (atom {}))

(defn lookup-phase [id]
  (if-some [vr (get @*phase-registry id)]
    @vr
    (throw (ex-info (str "Could not resolve phase: " (pr-str id)) {}))))

(defmacro defphase
  "Registers a phase (a state of the state machine).
  The phase will be registered with an ID that is a keyword
  of the same name as the specified var name.
  A docstring may be provided after the var symbol.
  Options that can be specified:
  :tick-fn (compulsory)
    A function that takes a single argument of the robot information
    where the value at :state is the phase state. Other useful keys
    include :readings, :input, :merged-state, :readings-history
  :init
    A map of the initial state, or a function that initialises it from a
    single map argument. Even if declaring as a map, the expression will
    be evaluated each time the phase is initialised.
  :sub-phases
    A map to declare sub-phases that will be nested in this phase.
  :chain
    A vector of sub-phases that are meant to be run in sequence."
  [phase-sym & args]
  (let [?doc (first args)
        ?doc (when (string? ?doc) ?doc)
        [& {:keys [init tick sub-phases chain]}] (cond-> args ?doc next)
        id (keyword phase-sym)
        statedecl->fn (fn [label decl]
                        (let [sym (symbol (str phase-sym ":" label))]
                          (if (and (seq? decl)
                                (= 'fn (first decl)))
                            (list* 'fn sym (next decl))
                            `(fn ~sym [_#] ~decl))))]
    `(swap! *phase-registry assoc ~id
       (def ~phase-sym
         ~@(when ?doc [?doc])
         ~(cond->
            {:id id
             :initial-state-fn (if init (statedecl->fn "init" init) {})
             :tick-fn `(fn ~(symbol (str phase-sym ":tick")) ~@(rest tick))}
            sub-phases
            (assoc :sub-phases-fn (statedecl->fn "sub-phases" sub-phases))
            chain
            (assoc :chain-fn (statedecl->fn "chain" chain)))))))

(defn get-initial-state
  ([phase] (get-initial-state phase nil))
  ([phase params]
   (let [{:keys [initial-state-fn
                 initial-state
                 sub-phases-fn
                 chain-fn
                 id]} phase
         state (or initial-state
                 (initial-state-fn params))
         subphases
         (when sub-phases-fn
           (update-vals
             (doto (sub-phases-fn params)
               (as-> m (assert (map? m)
                         (str "Phase-id: " (:id phase)
                           "\ngot: " (pr-str m)
                           "\nFor params: " (pr-str params)))))
             (fn [[phase params merged-state]]
               (merge (get-initial-state phase params)
                 merged-state))))]
     (cond-> (assoc state :phase-id id)
       subphases
       (assoc :sub-phases subphases)
       chain-fn
       (assoc :phase/chain
         (let [chain-decl (chain-fn params)]
           {:current-id 0
            :next-phase-map
            (conj (vec (map-indexed (fn [idx decl]
                                      (assoc decl 0 (inc idx)))
                         (drop 1 chain-decl)))
              ;; return nil for final phase
              nil)
            :id->phase-id (mapv (fn [[phase]]
                                  (:id phase))
                            chain-decl)
            :phases {0 (let [[phase params] (nth chain-decl 0)]
                         (get-initial-state phase params))}}))))))

(defn initialise-state [phase state]
  (merge state (get-initial-state phase)))

(defn mark-done [cmd]
  (update cmd :state assoc :phase/done? true))

(defn phase-done?
  ([{:keys [state]} nest-key]
   (phase-done? {:state (enc/have map?
                          (get-in state [:sub-phases nest-key]))}))
  ([{:keys [state]}]
   (:phase/done? state)))

(defn merge-states [state1 state2]
  (cond-> (merge state1 (dissoc state2 :sub-phases))
    (:sub-phases state2)
    (assoc :sub-phases
      (merge-with merge (:sub-phases state1) (:sub-phases state2)))))

(defn tick-phase [phase robot]
  ((:tick-fn phase) (assoc robot :phase phase)))

(defn tick-subphase [robot nest-key]
  (let [parent-state (:state robot)
        {:keys [sub-phases]} parent-state
        _ (assert (contains? sub-phases nest-key)
            (str nest-key " does not exist in " (pr-str (:phase-id parent-state))))
        sub-state (-> sub-phases nest-key)
        _ (assert (map? sub-state))
        nested-cmd (tick-phase (lookup-phase (:phase-id sub-state))
                    (assoc robot
                      :state sub-state
                      :merged-state (merge (:merged-state robot) sub-state)))]
    (update nested-cmd :state
      (fn [sub-state2]
        {:sub-phases
         (assoc {}
           nest-key (merge-states sub-state sub-state2))}))))

(defn merge-cmds
  "Merges the results of phases"
  ([cmd] cmd)
  ([cmd1 cmd2]
   (-> (merge-with merge
         cmd1 (dissoc cmd2 :state :readings-history))
     (assoc :state (merge-states (:state cmd1) (:state cmd2)))
     (merge (select-keys cmd2 [:readings-history]))))
  ([cmd1 cmd2 & cmds]
   (reduce merge-cmds
     (merge-cmds cmd1 cmd2)
     cmds)))

(defn tick-mapped-phase-group
  "Handles a group of phases where a mapping is specified
  that defines what phase comes after the phase that completed.
  After a phase completees, the tick function of the next
  phase will be executed within the same cycle (without needing
  to call this function again)."
  [robot state-key post-tick-fn]
  (let [parent-state (:state robot)
        {:keys [current-id phases next-phase-map]
         :as group-state}
        (state-key parent-state)
        substate1 (get phases current-id)
        merged-state (:merged-state robot)
        id->phase-id (:id->phase-id group-state identity)
        current-phase-id (:phase-id substate1
                           (id->phase-id current-id))
        current-phase (lookup-phase current-phase-id)
        substate1
        (or substate1
          (get-initial-state current-phase
            (merge merged-state (:init-params robot))))
        sub-cmd
          (tick-phase current-phase
            (assoc robot
              :state substate1
              :merged-state (merge merged-state substate1)))
        cmd (post-tick-fn robot
              (merge-cmds
                (dissoc sub-cmd :state)
                {:state {state-key
                         (assoc-in group-state [:phases current-id]
                           (merge-states substate1 (:state sub-cmd)))}}))
        robot2 (merge-cmds robot cmd)]
    (if-some [next-id-decl (if (phase-done? sub-cmd)
                              (next-phase-map current-id)
                              current-id)]
      (let [[next-id init-params]
            (if (vector? next-id-decl)
              next-id-decl
              [next-id-decl nil])]
        (cond
          ;; if transitioning to a new phase, run it immediately
          (not= current-id next-id)
          (let [robot2 (-> robot2
                         (assoc-in [:state state-key :current-id] next-id)
                         (update-in [:state state-key :phases] dissoc current-id))
                next-phase-id (id->phase-id next-id)]
            (println "Transition: " current-id
              (when (not= current-id current-phase-id)
                (str " (" current-phase-id ")")) " -> " next-id
              (when (not= next-id next-phase-id)
                (str " (" next-phase-id ")")))
            (recur
              (assoc robot2
                :merged-state (merge merged-state (:state robot2))
                :init-params init-params)
              state-key
              post-tick-fn))
          :else
          robot2))
      ;; Done if no subsequent phase is mapped
      (mark-done robot2))))

(defn tick-chain
  "Handles the specified :chain of sub-phases"
  ([robot] (tick-chain robot nil))
  ([robot {:keys [post-tick-fn]}]
   (tick-mapped-phase-group robot :phase/chain
     (or post-tick-fn (fn [_ cmd] cmd)))))