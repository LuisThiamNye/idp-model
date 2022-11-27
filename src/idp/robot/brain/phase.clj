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
        [& {:keys [init tick]}] (cond-> args ?doc next)
        id (keyword sym)]
    `(swap! *phase-registry assoc ~id
       (def ~sym
         ~@(when ?doc [?doc])
         {:id ~id
          :initial-state-fn
          ~(if (and (seq? init)
                 (= 'fn (first init)))
             init
             `(fn [_#] ~init))
          :tick-fn (fn ~sym ~@(rest tick))}))))

(defn get-initial-state
  ([phase] (get-initial-state phase nil))
  ([phase prev-state]
   (let [{:keys [initial-state-fn
                 initial-state]} phase]
     (or initial-state
       (initial-state-fn prev-state)))))

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
   (phase-done? {:state (get-in state [:sub-states nest-key])}))
  ([{:keys [state]}]
   (and (contains? state :phase-id)
     (nil? (:phase-id state)))))

(defn merge-states [state1 state2]
  (cond-> (merge state1 (dissoc state2 :sub-states))
    (:sub-states state2)
    (assoc :sub-states
      (merge-with merge (:sub-states state1) (:sub-states state2)))))

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
