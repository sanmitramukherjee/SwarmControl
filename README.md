# SwarmControl — Multi-Drone Ground Control Station

A multi-drone Ground Control Station (GCS) and Swarm Command Center for ArduPilot/PX4 UAVs over MAVLink. Features single-port serial telemetry multiplexing, a 3D Web Dashboard (Three.js), an optional Tkinter cockpit UI, leader-follower formations, collision avoidance, and autonomous one-click mission dispatch.

---

## System Requirements & Quick Setup

- **Python**: 3.8+
- **Automated Setup (Windows)**:
  Simply double-click `setup_venv.bat` (or run it from PowerShell / CMD). It will automatically create a virtual environment (`venv`), activate it, and install all required packages.

- **Manual Setup**:
  ```bash
  python -m venv venv
  .\venv\Scripts\activate
  pip install -r requirements.txt
  ```
- **Hardware Telemetry Link (for Real Drones)**:
  - Telemetry radio modem (e.g. SiK Radio, Holybro Telemetry, RFD900) plugged into your USB/COM port.
  - Multi-drone setups require each flight controller configured with a unique MAVLink `SYSID_THISMAV` (e.g., Drone 1 = `1`, Drone 2 = `2`) communicating over the same air frequency/baud rate.

---

## CLI Options & Usage Guide

```text
python DroneContUI.py [--sitl | --real] --drones SPEC [SPEC ...] [OPTIONS]
```

### Argument Reference

| Flag | Type | Description |
| :--- | :--- | :--- |
| `--real` | Flag | Connects to physical drones over serial / telemetry COM ports. |
| `--sitl` | Flag | Connects to ArduPilot SITL simulator instances over TCP localhost. |
| `--drones` | List | Drone targets: <br>• **Real Mode**: `COM_PORT:SYSID` (e.g., `COM3:1 COM3:2`)<br>• **SITL Mode**: TCP port numbers (e.g., `5762 5772`) |
| `--baud` | Int | Baud rate for serial connections (default: `57600`). |
| `--control` | Flag | Launches the legacy desktop Tkinter PFD cockpit window alongside the web server. |
| `--no-browser` | Flag | Prevents the default browser from automatically popping up on launch. |

---

## Operational Workflows

### 1. Real Drones (2 Drones over a Single COM Port)

When flying 2 physical drones over one telemetry radio (e.g., `COM3`):
- Ensure Drone 1 is set to `SYSID_THISMAV = 1`.
- Ensure Drone 2 is set to `SYSID_THISMAV = 2`.
- Telemetry radio baud is set to `57600` (or `115200` depending on radio config).

#### Command (Standard Web Dashboard):
```bash
python DroneContUI.py --real --drones COM3:1 COM3:2 --baud 57600
```

#### Command (With Desktop Tkinter Cockpit UI):
```bash
python DroneContUI.py --real --drones COM3:1 COM3:2 --baud 57600 --control
```

#### Command (Headless Server / No Auto-Browser):
```bash
python DroneContUI.py --real --drones COM3:1 COM3:2 --baud 57600 --no-browser
```

---

### 2. SITL Simulator Drones

Run ArduPilot SITL instances locally (e.g., instance 0 on port 5762, instance 1 on port 5772):

#### Command:
```bash
python DroneContUI.py --sitl --drones 5762 5772
```

#### Command (With Tkinter GUI):
```bash
python DroneContUI.py --sitl --drones 5762 5772 --control
```

---

## Swarm Flight Operation Workflow

### Web Dashboard Interface (`http://localhost:8000/index.html`)

1. **Dashboard Overview**:
   - **Leader**: Defaults to Drone 1 (`#1`).
   - **Proximity Alert**: Active at `< 1.5m`. Drones turn red with an onscreen collision alert and vertical stagger is applied automatically if waypoints converge.
   - **Altitude Reference**: All flight commands strictly use **Relative Altitude (AGL)** relative to each drone's home takeoff point.

2. **One-Click Autonomous Mission (`FLY TO TARGET`)**:
   - In the right-side **Swarm Control** panel:
     - Set **Formation**: `OFF`, `V-SHAPE`, or `LINE`.
     - Set **Spacing**: Desired distance in metres between drones (e.g. `10m`).
     - Enter target **LAT**, **LON**, and **ALT AGL (m)** (e.g. `10m`).
     - Click **FLY TO TARGET**.
   - **Autonomous Lifecycle**:
     1. **Ground Check**: If drones are on the ground/disarmed, the system automatically sets `GUIDED` mode, arms, and initiates a staggered takeoff.
     2. **Climb Verification**: Telemetry is monitored until all drones reach hover altitude (`target_alt - 1.0m`).
     3. **Formation Trajectory**: Follower coordinates are computed based on Drone 1's heading and formation geometry.
     4. **Arrival & Hover**: Commands are dispatched; progress is tracked until the swarm arrives within 2.5m of the targets.

3. **30-Second Inactivity Auto-RTL**:
   - While the swarm is actively taking off, climbing, or navigating, the watchdog is held in flight mode and **will not fire**.
   - Once all drones arrive at destination and enter hover with no new instructions for **30 seconds**, the swarm will automatically initiate **Return To Launch (RTL)** for safety.
   - Any manual interaction (new coordinates, clicking ARM/RTL/LAND/DISARM) immediately resets the inactivity timer.

4. **Emergency Controls**:
   - **RTL**: Commands all drones to Return to Launch immediately.
   - **LAND**: Commands all drones to land in place immediately.
   - **DISARM (EMERGENCY)**: Emergency disarm override.
