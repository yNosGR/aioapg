## All In One Autopilot Gateway
With an ESP32, we should be able to:
1. Gateway the N2k bus into an async TCP stream for use in OpenCPN, Navionics, etc.
2. Setup a few nmea0183 ports to mux into the N2K bus, an onboard GPS for example.
3. Output the 9dof info over an async stream for use in an dead reckoning based Inertial Navigation System.
4. Use the above info to drive a motor controller, creating an autopilot.
5. Serve up some javascript that will display the above info, possibly integrating https://github.com/mxtommy/Kip
