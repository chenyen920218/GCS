# PyQt5 UAV Ground Control Station (GCS)

A modern **Python-based Ground Control Station (GCS)** developed using **PyQt5** for UAV monitoring, mission planning, telemetry visualization, flight control, and real-time video streaming.

This project supports both **MAVLink communication** with real flight controllers and a **FakeLink simulation mode** for UI testing and software development without UAV hardware.

---

# Features

## Flight Telemetry

The GCS continuously monitors and displays flight information including:

- Roll
- Pitch
- Yaw
- Altitude
- Airspeed
- GPS position
- Battery status
- GPS satellites
- Flight mode
- Heartbeat status

Telemetry is updated in real time through MAVLink messages.

---

## MAVLink Communication

Supports MAVLink communication through:

### Serial Connection

Example

```
COM5
/dev/ttyUSB0
```

### UDP Connection

Example

```
udp:0.0.0.0:14550
```

Supported MAVLink messages include

- HEARTBEAT
- ATTITUDE
- GLOBAL_POSITION_INT
- VFR_HUD
- GPS_RAW_INT
- SYS_STATUS

The software automatically requests telemetry stream rates after a successful connection.

---

## FakeLink Simulation

A built-in simulator generates realistic flight data including

- Circular GPS trajectory
- Roll/Pitch/Yaw motion
- Altitude variation
- Airspeed variation
- Battery discharge
- GPS satellite changes
- Flight mode switching

This mode allows complete UI testing without connecting to an autopilot.

---

# Artificial Horizon

The application includes a custom-designed attitude indicator featuring

- Sky/Ground display
- Roll indicator
- Pitch ladder
- Roll scale
- Aircraft reference symbol
- Slip indicator

The widget is implemented entirely using Qt's QPainter.

---

# Live Flight Monitoring

A dedicated monitoring window provides

### Analog Gauges

- Altitude
- Airspeed
- Temperature

### Real-Time Plots

Generated using **PyQtGraph**

Displays

- Altitude history
- Airspeed history
- Temperature history

---

# Flight Path Map

The GCS integrates **Leaflet.js** with **OpenStreetMap**.

Functions include

- Live UAV position
- Automatic map centering
- Flight trajectory
- Track reset

The map is displayed using Qt WebEngine.

---

# Mission Planner

Mission planning interface supports

- Manual waypoint creation
- Waypoint deletion
- Clear all waypoints
- Upload mission to flight controller

Waypoints include

- Latitude
- Longitude
- Relative altitude

Mission upload uses MAVLink Mission protocol.

---

# Flight Commands

The GCS can send common flight commands including

- ARM
- DISARM
- TAKEOFF
- LAND
- Flight Mode Switching

Supported flight modes include

- MANUAL
- STABILIZE
- LOITER
- AUTO
- RTL

---

# Video Streaming

Supports receiving real-time images from an onboard Raspberry Pi.

Features

- UDP image transmission
- JPEG decoding
- Live display
- Compatible with YOLOv8 detection output

Default UDP port

```
5600
```

---

# Flight Data Logging

The application can record telemetry into CSV files.

Typical logged data include

- Time
- Roll
- Pitch
- Yaw
- Altitude
- Airspeed
- Latitude
- Longitude
- Temperature
- Battery
- GPS satellites
- Flight mode

The recorded logs can later be replayed.

---

# Flight Log Replay

Replay recorded flight logs with synchronized plots.

Displays

- Altitude
- Airspeed
- Temperature

Playback controls include

- Play
- Pause

---

# Software Architecture

```
GroundStationWindow
│
├── Telemetry
│
├── FakeLink
│
├── MavlinkLink
│
├── FlightChartsWindow
│
├── ReplayWindow
│
├── MissionPlannerWindow
│
├── VideoWindow
│
├── AttitudeWidget
│
├── GaugeWidget
│
└── WebEngineMapWidget
```

---

# Project Structure

```
.
├── GCS_v5_udp_modified_v5.py
├── map.html
├── README.md
└── logs/
```

---

# Requirements

Python 3.9+

Required packages

```
PyQt5
PyQtWebEngine
numpy
opencv-python
pyqtgraph
matplotlib
pyquaternion
pymavlink
```

Install using

```bash
pip install PyQt5
pip install PyQtWebEngine
pip install numpy
pip install opencv-python
pip install pyqtgraph
pip install matplotlib
pip install pyquaternion
pip install pymavlink
```

Or

```bash
pip install -r requirements.txt
```

---

# Running

Simply execute

```bash
python GCS_v5_udp_modified_v5.py
```

The application automatically generates

```
map.html
```

if it does not already exist.

---

# Connection Workflow

## FakeLink

```
Launch
      ↓
Select FakeLink
      ↓
Connect
      ↓
Generate simulated telemetry
```

## MAVLink

```
Launch
      ↓
Select Serial or UDP
      ↓
Connect
      ↓
Wait for Heartbeat
      ↓
Receive telemetry
```

---

# Typical Workflow

1. Start the Ground Control Station
2. Connect to the UAV (Serial/UDP) or FakeLink
3. Monitor live telemetry
4. View aircraft position on the map
5. Send ARM/Takeoff/Land commands
6. Upload mission waypoints
7. Record flight data
8. Replay recorded logs
9. View onboard video stream

---

# Technologies

- Python
- PyQt5
- Qt WebEngine
- MAVLink
- pymavlink
- Leaflet.js
- OpenStreetMap
- OpenCV
- NumPy
- PyQtGraph
- Matplotlib

---

# Future Improvements

Potential future enhancements include

- Mission editing on the map
- Drag-and-drop waypoint modification
- MAVLink parameter management
- Terrain visualization
- Offline map support
- RTSP/H.264 video streaming
- Multi-UAV support
- Joystick integration
- Object tracking
- ROS2 integration
- PX4 and ArduPilot compatibility improvements
- 3D flight visualization
- Health monitoring dashboard

---

# License

This project is intended for academic research and UAV development purposes.

Users are responsible for ensuring safe UAV operation and compliance with local aviation regulations.
