(ns idp.robot.brain.tunnel
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [clojure.core.match :refer [match]]
    [idp.robot.params :as robot.params]
    [idp.board.params :as board.params]
    [chic.util.ns :refer [inherit-vars]]
    [idp.robot.state :as rs]
    [idp.robot.brain.util :as bu]
    [idp.robot.brain.follow :as follow]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input
  bu/get-combined-line-readings
  bu/tracking-prolonged-condition
  bu/until-straight
  bu/until-turning
  bu/junction-turn-spin
  bu/straight-up-to-blackout
  follow/biased-follow
  follow/basic-follow
  follow/follow-up-to-blackout)

(defphase through-tunnel
  "Waits until refinding the line.
  Drives robot in straight line at constant speed.
  Uses side-mounted ultrasonic sensors to adjust direction."
  :init {:nfinds 0
         :min-duration 1000
         :duration 0}
  :tick
  (fn [{:keys [state readings]}]
    (let [min-finds 3 ;; require multiple line encounters for extra certainty
          speed 255
          found-line? (and
                        (<= (:min-duration state) (:duration state))
                        (match (get-combined-line-readings readings)
                         [:w  _ :b :b] true
                         [ _ :w :b :b] true
                         [:b :w  _ :b] true
                         [:b  _ :w :b] true
                         [:b :b :w  _] true
                         [:b :b  _ :w] true
                         :else false))
          nfinds (cond-> (:nfinds state) found-line? inc)]
      (if (<= min-finds nfinds)
        (phase/mark-done {:input {:ultrasonic-active? false}})
        (let [;; ultrasonic-based correction algorithm as suggested by Greg
              turn-speed
              (let [max-x-error 10 ;; mm
                    correcting-turn-speed 100
                    ;; Use small angle approximations (robot going mostly straight)
                    us-x-offset (-> robot.params/dims :ultrasonic-1 :pos :x)
                    us-left-dist (rs/get-left-ultrasonic readings)
                    deviation
                    (and (< 0 us-left-dist 500)
                      (let [;; estimated horizontal position of robot relative to tunnel midpoint
                            x-pos (- us-left-dist
                                    us-x-offset (/ (:tunnel-width board.params/dims) 2))
                            abs-deviation (- (abs x-pos) max-x-error)]
                        (when (pos? abs-deviation)
                          (cond-> abs-deviation (neg? x-pos) -))))]
                (if deviation
                  (cond-> correcting-turn-speed (pos? deviation) -)
                  0))]
          {:state {:nfinds nfinds
                   :duration (+ (:duration state) (rs/get-active-dt readings))}
           :input (assoc (motor-input speed turn-speed)
                    :ultrasonic-active? true
                    :grabber-position :open)})))))

(defphase tunnel-approach
  "Does line following up to tunnel.
  Tunnel starts after observing a certain number of line sensor
  blackouts, and when ultrasonic sensor reads a reasonable distance."
  :init {}
  :sub-phases
  {:follow [basic-follow]
   :follow-b [follow-up-to-blackout]
   :us-condition
   [tracking-prolonged-condition
    {:min-duration 100
     :pred
     (fn [{:keys [readings]}]
       (< 0 (rs/get-left-ultrasonic readings) 150))}]}
  :tick
  (fn [{:as robot}]
    (let [cmd (phase/merge-cmds
                (phase/tick-subphase robot :us-condition)
                {:input {:grabber-position :open
                         :ultrasonic-active? true}})]
      (if (phase/phase-done? cmd :us-condition)
        (let [cmd (phase/tick-subphase robot :follow-b)]
          (if (phase/phase-done? cmd :follow-b)
            (phase/mark-done
              (phase/merge-cmds cmd
                {:input {:ultrasonic-active? false}}))
            cmd))
        (phase/merge-cmds cmd
          (phase/tick-subphase robot :follow))))))

(defphase up-to-tunnel-end
  :chain [[tunnel-approach]
          [through-tunnel]]
  :tick
  (fn [robot] (phase/tick-chain robot)))