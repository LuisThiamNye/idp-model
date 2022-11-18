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
          :initial-state ~init
          :tick-fn (fn ~sym ~@(rest tick))}))))

(defn initialise-state [phase state]
  (merge {:next-phase-map {}}
    state
    (assoc (:initial-state phase)
      :phase-id (:id phase))))

(defn initialise-phase-on-state [state phase]
  (initialise-state phase state))

(defn init-phase-id-on-state [state phase-id]
  (initialise-state (lookup-phase phase-id) state))
