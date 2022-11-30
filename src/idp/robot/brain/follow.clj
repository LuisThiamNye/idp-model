(ns idp.robot.brain.follow
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [chic.util.ns :refer [inherit-vars]]
    [clojure.core.match :refer [match]]
    [idp.robot.state :as rs]
    [idp.robot.brain.util :as bu]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input
  bu/get-combined-line-readings)

(defn tick-common-follow [{:keys [state readings] :as robot} follow-strategy]
  (let
    [intent ((:intent-fn follow-strategy) state readings)
     intent
     (if (and (= [:continue] intent)
           (not (:tolerate-blackout? follow-strategy)))
       (let [{:keys [readings-history]} robot]
         (when-some
           [last-nonblackout-readings
            (reduce (fn [_ readings]
                      (when (not= [:b :b :b :b]
                              (get-combined-line-readings readings))
                        (reduced readings)))
              nil (rseq readings-history))]
           ((:intent-fn follow-strategy)
            state last-nonblackout-readings)))
       intent)]
    (if (= [:continue] intent)
      {}
      (let [level-speeds (:level-speeds follow-strategy)
            [forward-speed turn-speed]
            (match intent
              [:straight] (nth level-speeds 0)
              [left-or-right level]
              (let [[forward turn] (nth level-speeds level)]
                [forward
                 (cond-> turn (= :left left-or-right) -)]))]
        {:state {:follow-intent intent}
         :input (motor-input forward-speed turn-speed)}))))

(def basic-follow-strategy
  {:intent-fn
   (fn [state readings]
     (let [combined-readings (get-combined-line-readings readings)
           prev-intent (:follow-intent state)]
       (match combined-readings
         [_ :w :w _] [:straight]
         [:b :w :b :b] [:left 1]
         [:w :w :b :b] [:left 2]
         [:w :b :b :b] [:left 3]
         [:b :b :w :b] [:right 1]
         [:b :b :w :w] [:right 2]
         [:b :b :b :w] [:right 3]
         [:b :b :b :b]
         (match prev-intent
           ;; go straight if line lost when between sensors
           ;; - important for ramp
           [:left 1]  [:straight]; [:left 2]
           [:left 3]  [:left 4]
           [:right 1] [:straight];[:right 2]
           [:right 3] [:right 4]
           :else prev-intent)
         :else prev-intent)))})

(defphase basic-follow
  "Follow the line, centred on the robot"
  :init {:follow-intent [:continue]
         :high-power? false}
  :tick
  (fn [{:keys [state] :as robot}]
    (tick-common-follow robot
      (assoc basic-follow-strategy :level-speeds
        (if (:high-power? state)
          [[255 0] [255 30] [255 30] [0 255] [0 255]]
          [[255 0] [230 30] [80 150] [20 200] [0 200]])))))

(def biased-follow-strategy
  {:intent-fn
   (fn [state readings]
     (let [combined-readings (get-combined-line-readings readings)
           mirror-intent #(update % 0
                            (some-fn
                              {:left :right
                               :right :left}
                              identity))
           ;; Calculate in terms of left bias
           swap? (case (:bias state) :right true :left false)
           prev-intent (cond-> (:follow-intent state [:straight])
                         swap? mirror-intent)
           {:keys [bias-level]} state
           intent
           (case (int bias-level)
             ; 1 ;; align line with 3rd sensor
             ; (match (cond-> combined-readings
             ;          swap? (-> rseq vec))
             ;   [ _ :w :w :w] [:straight]
             ;   [:b :b :w  _] [:straight]
             ;   [ _ :w :w :b] [:left 1]
             ;   [ _ :w :b :b] [:left 2]
             ;   [:w :b :b :b] [:left 3]
             ;   [ _  _ :b :w] [:right 1]
             ;   [:b :b :b :b]
             ;   (match prev-intent
             ;     [:right 1] [:right 4]
             ;     :else prev-intent)
             ;   :else prev-intent)
             2 ;; keep line between sensor 3 and 4
             (match (cond-> combined-readings
                      swap? (-> rseq vec))
               [ _  _ :w :w] [:straight]
               [:b :b :w :b] [:left 1]
               [:b :b :b :w] [:right 1]
               [ _ :w :w :b] [:left 3]
               [ _ :w :b :b] [:left 4]
               [:w :b :b :b] [:left 4]
               [ _  _ :b :w] [:right 2]
               [:b :b :b :b]
               (match prev-intent
                 [:left 1] [:straight]
                 [:left 2] [:straight]
                 [:left 3] [:left 4]
                 [:right 1] [:right 4]
                 [:right 2] [:right 4]
                 [:right 3] [:right 4]
                 :else prev-intent)
               :else prev-intent))]
       (cond-> intent swap? mirror-intent)))})

(defphase biased-follow
  "Follow the line, keeping the line aligned with either the
  middle- left or right sensor"
  :init (fn [{:keys [bias] :as params}]
          {:bias (enc/have #{:left :right} bias)
           :bias-level (enc/have #{2} (:bias-level params 2))
           :high-power? false
           :follow-intent [:continue]
           })
  :tick
  (fn [{:keys [state] :as robot}]
    (tick-common-follow robot
      (assoc biased-follow-strategy :level-speeds
        (if (:high-power? state)
          [[255 0] [255 30] [255 30] [0 255] [0 255]]
          [[255 0] [230 30] [80 150] [20 200] [0 180]])))))

(defphase follow-up-to-blackout
  "Does line following to tightly follow the line until it suddenly ends.
  Requires a certain number of 'blackout' readings to end.
  Assumes there is no 'dead zone' between any of the line sensors."
  :init {:follow-intent [:continue]
         :blackout-duration 0}
  :tick
  (fn [{:keys [state readings] :as robot}]
    (let
      [min-blackouts-duration 400 ;; determines when phase is done (ms)
       blackout-speed 150 ;; after first blackout, go slower
       combined-readings (get-combined-line-readings readings)
       {:keys [blackout-duration]} state
       max-fspeed (if (pos? blackout-duration) blackout-speed 230)
       cmd (tick-common-follow robot
             {:tolerate-blackout? true
              :level-speeds [[max-fspeed 0]
                             [(* max-fspeed 0.9) 35]
                             nil
                             nil
                             [0 200]]
              :intent-fn
              (fn [state _readings]
                (let [prev-intent (:follow-intent state)]
                  (match combined-readings
                    [_ :w :w _] [:straight]
                    [:b :w :b :b] [:left 1]
                    [:b :b :w :b] [:right 1]
                    [:w _ :b :b] [:left 4]
                    [:b :b _ :w] [:right 4]
                    [:b :b :b :b]
                    (match prev-intent
                      [:left 1]  [:straight]
                      [:right 1] [:straight]
                      :else prev-intent)
                    :else prev-intent)))})]
      (if (<= min-blackouts-duration blackout-duration)
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          {:state {:blackout-duration
                   (if (= [:b :b :b :b] combined-readings)
                     (+ blackout-duration (rs/get-active-dt readings))
                     0)}})))))

;; TODO FIND LINE