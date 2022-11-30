(ns idp.robot.brain.junction
  (:require
    [taoensso.encore :as enc]
    [idp.robot.brain.phase :as phase :refer [defphase]]
    [idp.robot.brain.util :as bu]
    [chic.util.ns :refer [inherit-vars]]
    [clojure.core.match :refer [match]]
    [idp.robot.brain.follow :as follow]
    [idp.robot.state :as rs]))

(inherit-vars
  bu/get-line-triggers
  bu/timed-straight
  bu/motor-input
  bu/get-combined-line-readings
  bu/tracking-prolonged-condition
  bu/until-straight
  bu/until-turning
  bu/junction-turn-spin
  bu/set-input
  bu/tracking-motor-forward
  bu/straight-up-to-blackout
  bu/tracking-motor-forward
  follow/biased-follow
  follow/basic-follow)

(defphase junction-approach-turn-excess
  "Moves robot forwards, from position where line sensors are at the junction
  path, to where the centre of rotation is approximately aligned with
  the path of a box"
  :init (fn [{:keys [turn-direction] :as params}]
          {:status :pre-align
           :excess-coeff (:excess-coeff params 1)
           :turn-direction turn-direction})
  :sub-phases
  (fn [{:keys [turn-direction]}]
    {:forward-amount [tracking-motor-forward]
     :follow [biased-follow {:bias (case turn-direction
                                     :right :left
                                     :left :right)}]})
  :tick
  (fn [{:keys [readings]
        {:keys [status turn-direction excess-coeff]} :state
        :as robot}]
    (let [min-fwd-amount (long (* excess-coeff 70e4)) ;; MAGIC NUMBER
          cmd (phase/tick-subphase robot :forward-amount)
          forward-amount (-> cmd :state :sub-phases :forward-amount :forward-amount)
          cmd (phase/merge-cmds cmd
                (phase/tick-subphase robot :follow))]
      (if (<= min-fwd-amount forward-amount)
        (phase/mark-done cmd)
        (phase/merge-cmds cmd
          (when (= :pre-align status)
            ;; initially turn away from box a bit to avoid following the path up to it
            (let [ls (get-combined-line-readings readings)
                  nwhites (count (filterv #(= :w %) ls))
                  aligned? (<= nwhites 1)]
              (if aligned?
                {:state {:status :follow}}
                {:input (motor-input 0 (cond-> 200 (= :right turn-direction) -))}))))))))

(defphase junction-approach-turn
  :chain
  (fn [{:keys [turn-direction] :as params}]
    (enc/have? #{:left :right} turn-direction)
    [[junction-approach-turn-excess
      {:turn-direction turn-direction
       :excess-coeff (:excess-coeff params 1)}]
     [junction-turn-spin {:turn-direction turn-direction
                          :forward-speed (:turn-forward-speed params 0)}]])
  :tick
  (fn [robot] (phase/tick-chain robot)))

(defphase up-to-junction
  "Goes from somewhere on the main line up to one of the
  three junctions of a line box or collection point.
  Performs biased line following to ensure junction detection."
  :init (fn [{:keys [junction-number turn-direction]}]
          {:junction-number (enc/have #{1 2 3} junction-number)
           :on-junction? false
           :nfinds 0
           :turn-direction turn-direction})
  :sub-phases
  (fn [{:keys [turn-direction]}]
    {:follow [biased-follow {:bias (enc/have #{:left :right} turn-direction)}]})
  :tick
  (fn [{:keys [state readings]
        {:keys [turn-direction]} :state :as robot}]
    (let [prev-on-junction? (:on-junction? state)
          combined-line-readings (get-combined-line-readings readings)
          ;; from perspective of turning right
          normalised-ls (cond-> combined-line-readings
                          (= :left (:turn-direction state))
                          (-> rseq vec))
          on-junction? (if prev-on-junction?
                          (match normalised-ls
                            [ _  _ :b :b] false
                            [:b :b :w :b] false
                            :else true)
                          (match normalised-ls
                            [ _  _ :w :w] true
                            [:w :w :w  _] true
                            :else false))
          nfinds (cond-> (:nfinds state)
                   (and (not on-junction?)
                     (not= prev-on-junction? on-junction?))
                   inc)
          {:keys [junction-number]} state
          cmd (cond-> (phase/tick-subphase robot :follow)
                ;; avoid following box path when at junction
                on-junction?
                (phase/merge-cmds
                  {:input (motor-input 180 (cond-> 80 (= :right turn-direction) -))}))]
      (if (<= junction-number nfinds)
        (phase/mark-done cmd)
        (phase/merge-cmds
          cmd
          {:state {:on-junction? on-junction?
                   :nfinds nfinds}})))))