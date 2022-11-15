(ns idp.main
  (:require
    [taoensso.encore :as enc]
    [babashka.fs :as fs]
    [clojure.main]
    [nrepl.core :as nrepl]
    [nrepl.cmdline]
    [nrepl.server]
    [idp.uiroot :as uiroot]
    [idp.sim]
    [idp.robot.autopilot]
    [idp.robot.monitor.panel :as monitor.panel]
    [chic.debug.nrepl :as debug.nrepl]))

;; (set! *warn-on-reflection* true)

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

(def *threads (atom {}))

(defn on-shutdown []
  (fs/delete-if-exists ".nrepl-port"))

(defn start-app! []
  (idp.uiroot/start-ui!))

(defn -main [& args]
  (start-nrepl-server!)
  (.addShutdownHook (Runtime/getRuntime)
    (Thread. #'on-shutdown))
  (when (not-any? #{"nostart"} args) (start-app!))
  (println "Initialised"))

(comment
  (monitor.panel/open-window-safe!)
  (uiroot/open-sim-window!)
  
  (start-app!)
  (start-nrepl-server!)
  (debug.nrepl/add-middleware (nrepl/client nrepl-transport 1000))
  
  )
