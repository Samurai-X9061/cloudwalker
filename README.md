# CloudWalker - GPS-Denied Autonomous Drone

## Things to be done/remembered
- MAVSDK Library for interface with rpi and use MAVLink passthrough api for sending flow data (format is how pymavlink does it)
- Flow data needs to be corrected for roll and pitch using a gyro attached onto the rpi
- Failsafe to switch from guided to manual, have rc set at highest priority and have mode switches, timeout
