(ns idp.robot.sim.device
  "Behaviour of the simulated robot hardware"
  (:require
    [idp.robot.params :as robot.params]
    [idp.robot.state :as robot.state]))

(defn motor-speed->coeff
  "Normalises a [-255, 255] speed to a [-1,1] coefficient.
  Also simulates reduction in actual motor speed due to friction"
  [spd]
  (let [spd (/ spd 255.)
        motor-offset 0.16]
    (if (neg? spd)
      (max -1 (min 0 (+ spd motor-offset)))
      (min 1 (max 0 (- spd motor-offset))))))

(defn process-input!
  "Applies input commands to actual state of simulated robot"
  [input]
  (let
    [{:keys [motor-1 motor-2]} input
     {:keys [max-rpm wheel-diameter wheel-spacing]}
     robot.params/dims
     motor->v
     (fn [spd]
       (let
         [rpm (* max-rpm
                (motor-speed->coeff spd))
                rps (/ rpm 60)
                circumference (* Math/PI wheel-diameter)]
         (* circumference rps)))
     motor1-v (motor->v motor-1)
     motor2-v (motor->v motor-2)]
    (swap! robot.state/*real assoc
      :velocity (/ (+ motor1-v motor2-v) 2)
      :angular-velocity
      (* (/ (- motor1-v motor2-v)
           wheel-spacing)
        (/ 180 Math/PI)))))

(defn take-readings
  "Returns current intrinsic readings from the simulated robot"
  []
  (let [real-state @robot.state/*real]
    (->
      (select-keys real-state
        [:line-sensor-1
         :line-sensor-2
         :line-sensor-3
         :line-sensor-4])
      (assoc
        :block-density @robot.state/*sim-block-density
        :block-present? @robot.state/*sim-block-present?
        :ultrasonic-1 (:distance (:ultrasonic-1 real-state))
        :ultrasonic-2 (:distance (:ultrasonic-2 real-state))))))