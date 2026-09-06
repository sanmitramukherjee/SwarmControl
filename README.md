# SwarmControl GCS 

**SwarmPilot GCS** is a next-generation, web-based Ground Control Station (GCS) designed for the real-time visualization and management of drone swarms. Powered by a high-performance **Three.js** 3D engine and driven by the **PyMAVLINK** protocol, this application provides a dual-layer interface: a global 3D swarm visualizer and a granular 8-drone independent control dashboard, engineered to scale effortlessly up to **100+ drones**.
Supports both real & SITL (Software in the Loop) drones alike.

---

##  Key Features

###  3D Swarm Visualizer (Three.js)
* **Real-Time 3D Simulation:** View the entire drone swarm in a fully responsive, hardware-accelerated 3D environment.
* **Spatial Telemetry:** Live mapping of drone positions, altitudes, headings, and flight paths.
* **Dynamic Environment:** Support for custom terrain mapping and coordinate grid overlays.

###  Dual-Control Architecture
* **Primary Swarm Commander:** Broadcast global mission commands to the entire swarm simultaneously.
* **Secondary Multi-Drone Dashboard:** Dedicated, high-density UI layout to monitor and control up to **8 drones independently** on a single screen.
* **Massive Scalability:** Backend and frontend pipelines optimized to scale up to **100 drones** without UI lag or telemetry bottlenecks.

###  Protocol & MAVLink Commands
* **PyMAVLINK Integration:** Robust, industry-standard MAVLink communication layer.
* **Core Command Suite:** Execute critical commands instantly:
  * **Safety:** Arm / Disarm
  * **Flight:** Take Off / Land / Return to Launch (RTL)
  * **Navigation:** Fly To (Waypoint/Guided execution)

---

##  Tech Stack

* **Frontend UI:** HTML5, CSS3, JavaScript (ES6+), Bootstrap/Tailwind CSS
* **3D Graphics Engine:** Three.js (WebGL)
* **Backend Gateway:** Python (Flask-SocketIO / FastAPI)
* **Communication Protocol:** PyMAVLINK (MAVLink over UDP/TCP)

---

## 📈 Scalability Configuration

To scale the system up to 100 drones, optimize your network configuration in `server/config.json`:
* Increase the MAVLink message streaming rate parameters (`SRx_POSITION`, `SRx_EXTRA1`).
* Adjust the WebSocket throttling interval (default is set to 50ms) to prevent frontend thread locking.

---

## 🤝 Contributing

Contributions make the open-source community an amazing place to learn, inspire, and create. 
1. Fork the Project.
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`).
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the Branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.
