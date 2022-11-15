(ns idp.main
  (:require
    [babashka.fs :as fs]
    [nrepl.core :as nrepl]
    [nrepl.cmdline]
    [nrepl.server]
    [io.github.humbleui.ui :as ui]
    [chic.debug.nrepl :as debug.nrepl]))

(def nrepl-server nil)
(def nrepl-transport nil)

(defn start-nrepl-server! []
  (alter-var-root #'nrepl-server
    (fn [x]
      (when (some? x) (nrepl.server/stop-server x))
      (let [server (nrepl.server/start-server
                     ;:port 7888
                     {})]  
        (try
          (alter-var-root #'nrepl-transport
            (constantly (nrepl/connect :port (:port server))))
          (debug.nrepl/add-middleware (nrepl/client nrepl-transport 1000))
          (nrepl.cmdline/save-port-file server {})
          server
          (catch Throwable _
            (nrepl.server/stop-server server)
            nil))))))

(defn on-shutdown []
  (fs/delete-if-exists ".nrepl-port"))

(defn start-app! []
  (ui/start-app!)
  ((requiring-resolve 'idp.hub/open-hub-window!)))

(defn -main [& args]
  (start-nrepl-server!)
  (.addShutdownHook (Runtime/getRuntime)
    (Thread. #'on-shutdown))
  (when (some #{"reflection"} args)
    (println "Enabled *warn-on-reflection*")
    (set! *warn-on-reflection* true))
  (println "Starting application")
  (start-app!)
  (println "Initialised"))

