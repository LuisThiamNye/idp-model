
# Part IB Integrated Design Project

To run:

`clojure -M:dev`

Overview:

- Simulation of robot on the table.
- Autopilot for simulation and WiFi-connected robot.
- Live visualisations of sensor data.

The state machine of the autopilot is specified in the `idp.robot.brain` namespaces.

The autopilot loop is in `idp.robot.autopilot`.

`idp.robot.client` contains generic logic for communicating with the server, while `idp.net.api` contains specific code for encoding and decoding messages sent over the network.

The entry point of the program is in `idp.main`

---

Copyright 2022 Luis Thiam-Nye and contributors

