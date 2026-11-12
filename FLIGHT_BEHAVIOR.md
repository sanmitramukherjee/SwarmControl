# Leader sequencing and formation transitions

The selected leader determines dispatch order, regardless of its numeric system ID.

- Takeoff waits for the leader to report armed and airborne before launching followers.
- Land and RTL wait for each aircraft to report low altitude and disarmed before dispatching the next aircraft. The leader goes first. A timeout stops the sequence and is reported in the log.
- Emergency disarm dispatches immediately in leader-first order without waiting for touchdown.
- Manual follow observes the leader's landing before initiating follower landing.
- Autonomous formation targets share the requested AGL altitude. Manual follow uses the leader's observed AGL altitude. Formation-off navigation preserves horizontal offsets instead of stacking drones at one coordinate.
- RTL no longer overwrites the flight controller's configured return altitude.

Formation transitions use conservative horizontal path checks and sequential movement. Paths that cross another aircraft's occupied space are rejected, with current-position hold commands where applicable. This can reject a requested V transition rather than automatically find a detour. It also changes the timing of follower motion compared with simultaneous waypoint dispatch.

AGL is relative to each aircraft's home: equal AGL does not establish equal absolute height if home elevations differ. Grounded/disarmed detection uses the telemetry available in this project, rather than a dedicated flight-controller landed-state message.

Unit tests use simulated handlers. They do not establish physical collision avoidance, account for GPS error and wind, or certify behavior with an independently moving RC leader. Controlled SITL and hardware-in-the-loop flight validation remain necessary before field use. Restart the Python application to load these backend changes; reloading the browser only updates the frontend.
