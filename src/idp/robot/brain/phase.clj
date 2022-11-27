(ns idp.robot.brain.phase)

(def *phase-registry
  "Maps keyword to a phase definition var"
  (atom {}))

(defn lookup-phase [id]
  (if-some [vr (get @*phase-registry id)]
    @vr
    (throw (ex-info (str "Could not resolve phase: " (pr-str id)) {}))))

(defmacro defphase [sym & args]
  (let [?doc (first args)
        ?doc (when (string? ?doc) ?doc)
        [& {:keys [init tick sub-phases]}] (cond-> args ?doc next)
        id (keyword sym)
        statedecl->fn (fn [decl]
                        (if (and (seq? decl)
                              (= 'fn (first decl)))
                          decl
                          `(fn [_#] ~decl)))]
    `(swap! *phase-registry assoc ~id
       (def ~sym
         ~@(when ?doc [?doc])
         ~(cond->
            {:id id
             :initial-state-fn (statedecl->fn init)
             :tick-fn `(fn ~(symbol (str sym ":tick")) ~@(rest tick))}
            sub-phases
            (assoc :sub-phases-fn (statedecl->fn sub-phases)))))))

(defn get-initial-state
  ([phase] (get-initial-state phase nil))
  ([phase params]
   (let [{:keys [initial-state-fn
                 initial-state
                 sub-phases-fn]} phase
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
             (fn [[phase params]]
               (merge (get-initial-state phase params)
                 {:phase-id (:id phase)}))))]
     (cond-> state
       subphases
       (assoc :sub-phases subphases)))))

(defn initialise-state [phase state]
  (merge state
    (assoc (get-initial-state phase)
      :phase-id (:id phase))))

(defn initialise-phase-on-state [state phase]
  (initialise-state phase state))

(defn init-phase-id-on-state [state phase-id]
  (initialise-state (lookup-phase phase-id) state))

(defn mark-done [cmd]
  (update cmd :state assoc :phase-id nil))

(defn phase-done?
  ([{:keys [state]} nest-key]
   (phase-done? {:state (or (get-in state [:sub-states nest-key])
                          (get-in state [:sub-phases nest-key]))}))
  ([{:keys [state]}]
   (and (contains? state :phase-id)
     (nil? (:phase-id state)))))

(defn merge-states [state1 state2]
  (cond-> (merge state1 (dissoc state2 :sub-states :sub-phases))
    (:sub-states state2)
    (assoc :sub-states
      (merge-with merge (:sub-states state1) (:sub-states state2)))
    (:sub-phases state2)
    (assoc :sub-phases
      (merge-with merge (:sub-phases state1) (:sub-phases state2)))))

(defn tick-nested [phase nest-key robot]
  (let [parent-state (:state robot)
        {:keys [sub-states]} parent-state
        _ (assert (contains? sub-states nest-key)
            (str nest-key " does not exist"))
        sub-state (-> sub-states nest-key)
        nested-cmd ((:tick-fn phase) (assoc robot :state sub-state))]
    (update nested-cmd :state
      (fn [sub-state2]
        {:sub-states
         (assoc {}
           nest-key (merge-states sub-state sub-state2))}))))

(defn tick-subphase [robot nest-key]
  (let [parent-state (:state robot)
        {:keys [sub-phases]} parent-state
        _ (assert (contains? sub-phases nest-key)
            (str nest-key " does not exist in " (pr-str (:phase-id parent-state))))
        sub-state (-> sub-phases nest-key)
        _ (assert (map? sub-state))
        nested-cmd ((:tick-fn (lookup-phase (:phase-id sub-state)))
                    (assoc robot :state sub-state))]
    (update nested-cmd :state
      (fn [sub-state2]
        {:sub-phases
         (assoc {}
           nest-key (merge-states sub-state sub-state2))}))))

(defn get-nested-state [{:keys [state]} nest-key]
  (get (:sub-states state) nest-key))

(defn merge-cmds
  ([cmd1 cmd2]
   (-> (merge-with merge
         cmd1 (dissoc cmd2 :state))
     (assoc :state (merge-states (:state cmd1) (:state cmd2)))))
  ([cmd1 cmd2 & cmds]
   (reduce merge-cmds
     (merge-cmds cmd1 cmd2)
     cmds)))
