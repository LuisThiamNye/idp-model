(ns idp.state)

(def *window (promise))

(def *app (atom nil))

(def *misc (atom {:mouse-on-line? false}))