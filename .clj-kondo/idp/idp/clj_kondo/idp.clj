(ns clj-kondo.idp
  (:require [clj-kondo.hooks-api :as api]))

(defn defphase [{:keys [node]}]
  {:node (let [[_defphase sym ?docstr & body] (:children node)]
           (api/list-node
             (list* (api/token-node 'def) sym
               (if (api/string-node? ?docstr)
                 [?docstr (api/map-node body)]
                 [(api/map-node (cons ?docstr body))]))))})