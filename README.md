# Gesture Command Drones 🖐️🚁

Ever wondered what it feels like to command a drone with your hands?  
This project brings that fantasy to life — a gesture-controlled drone system where your **laptop processes gestures** and the **drone reacts via telemetry** in real-time.

---

## Features
- 🖐️ **Real-time hand gesture detection** using MediaPipe  
- 🚁 **Drone control via DroneKit & MAVLink**  
- 💻 **Processing done on your laptop** (no onboard heavy AI needed)  
- 🔄 **Works with both SITL (simulation) and real drones**  

---

## Quick Start

### 1. Clone the Repo
```bash
git clone https://github.com/ArjeeshChristal/Gesture-Command-Drones.git
cd Gesture-Command-Drones
```

### 2. Install Dependencies
```bash
pip install opencv-python mediapipe dronekit pymavlink
```

### 3. Run SITL (Optional but Recommended)
```bash
sim_vehicle.py -v ArduCopter --console --map
```

Change the connection string in `gesture_drone.py` to:
```python
connection_string = '127.0.0.1:5760'
```

### 4. Connection Check (Before Real Flight)
```bash
python3 connection_check.py
```

✅ Make sure the telemetry link works and the drone responds.

### 5. Fly Your Drone
* Update `connection_string` to your USB telemetry port (e.g., `/dev/ttyUSB0`)
* Start the gesture script:
```bash
python3 gesture_drone.py
```
* Move your hands in the hover zone — watch your drone obey!

---

## Pre-Flight Checklist

* Fully charged batteries (drone & telemetry)
* Secure propellers & frame screws
* RTL (Return-To-Launch) configured
* Open space free of people, pets, or obstacles
* Optional: Goggles & gloves

---

## Demo

Check the `demo/` folder for a video of SITL + real drone in action.

---

## License

MIT License © 2025 Arjeesh Christal

---

## Acknowledgements

* [DroneKit](https://dronekit-python.readthedocs.io/)
* [MediaPipe](https://google.github.io/mediapipe/)
* [ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
