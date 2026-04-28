
---

# Kihomac Uncrewed Underwater Vehicle

Embedded control system for an autonomous underwater vehicle (AUV) built on the Raspberry Pi Pico. This project implements a modular, real-time architecture for controlling buoyancy, propulsion, and vehicle stability in a constrained embedded environment.

---

## 🚀 Key Features

* **Non-blocking control loop** for real-time responsiveness
* **State-machine mission framework** for deterministic behavior
* **Modular test architecture** for rapid subsystem validation
* **Integrated sensor feedback** via MS5837 pressure sensor
* **Multi-actuator control** using PCA9685 PWM driver

---

## 🧠 Architecture

The system is structured around a centralized scheduler (`main.py`) and reusable test modules:

* **`main.py`** — Executes the main update loop and manages test lifecycle
* **`/tests/`** — Mission scripts implementing:

  * Ballast control (buoyancy)
  * Dive and cruise sequences
  * Fin-based control logic
  * Solenoid actuation

Each module follows a standardized interface:

```python
start() → initialize system
update() → non-blocking control loop
stop() → safe shutdown
```

This design enables **interruptible execution**, predictable timing, and clean extensibility.

---

## ⚙️ Hardware Integration

* **Controller:** Raspberry Pi Pico
* **PWM Driver:** PCA9685
* **Sensor:** MS5837 (pressure, depth, temperature)
* **Actuators:** Thruster ESC, ballast servo, control fins, solenoid valve

---

## 🔄 Control Strategy

* Time-driven state transitions (no blocking delays)
* Phase-based mission execution
* Open-loop fin control with planned PID upgrade path
* Real-time actuator updates within a continuous loop

---

## 📊 Data & Testing

* CSV logging of pressure, depth, and temperature
* Designed for iterative tuning and validation
* Modular tests allow isolated subsystem debugging

---

## 🛠️ Setup

```bash
git clone https://github.com/erikgoodwin-0413/Kihomac-Uncrewed-Underwater-Vehicle.git
```

Flash MicroPython to the Pico, upload files, and run `main.py`.

---

## 🔮 Next Steps

* Closed-loop PID depth and attitude control
* IMU integration for stabilization
* Sensor fusion and autonomy layer
* Safety watchdog (depth, leak, timeout)

---

## 📌 Summary

This project demonstrates **embedded systems design, real-time control, and hardware-software integration** in a mission-driven robotics application, with a focus on reliability, modularity, and iterative development.

---