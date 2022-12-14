(ns idp.main
  "Entry point of the program"
  (:require
    [babashka.fs :as fs]
    [nrepl.core :as nrepl]
    [nrepl.cmdline]
    [nrepl.server]
    [io.github.humbleui.ui :as ui]
    [chic.debug.nrepl :as debug.nrepl])
  (:import
    (java.util.concurrent Executors)))

;; Configure Clojure to use virtual threads for futures and agents
(set-agent-send-executor!
  (Executors/newVirtualThreadPerTaskExecutor))
(set-agent-send-off-executor!
  (Executors/newVirtualThreadPerTaskExecutor))

(def nrepl-server nil)
(def nrepl-transport nil)

(defn start-nrepl-server! []
  (alter-var-root #'nrepl-server
    (fn [x]
      (when (some? x) (nrepl.server/stop-server x))
      (let [server (nrepl.server/start-server {})]
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
  ;; Setup an nREPL server to allow connecting to the program
  ;; with an editor for dynamic evaluation of code
  (start-nrepl-server!)
  (.addShutdownHook (Runtime/getRuntime)
    (Thread. #'on-shutdown))
  
  (when (some #{"reflection"} args)
    (println "Enabled *warn-on-reflection*")
    (set! *warn-on-reflection* true))
  
  (println "Starting application")
  (start-app!)
  (println "Initialised")
  
  ;; Block the main thread so that program remains alive
  ;; Note that nREPL runs in a virtual (hence daemon) thread
  (let [o (Object.)]
    (locking o (.wait o))))

