# Rehab Gantry Application

A 3-axis Cartesian robotic gantry system designed for upper-limb motor rehabilitation. This project couples a robust, hardware-level **First-Order Admittance Control** engine with a Python-based supervisory UI, delivering engaging gamified therapies while ensuring absolute patient safety.

## 🚀 Key Features

* **Decoupled Architecture for Safety**: 
  * **The Damper (Firmware)**: The STM32 calculates a pure viscous damper response ($F = Bv$) at 1kHz. This prevents dangerous momentum buildup and ensures the robot stops instantly if the patient releases force.
  * **The Spring (Software)**: The Python game supervises gameplay and dynamically applies an **Assist-As-Needed (AAN)** algorithm, gently pulling the patient toward targets if they struggle.
* **Multi-Plane Gamification**: Supports dynamic game modes across different 2D planes of the 3D gantry space, seamlessly homing and locking unused axes.
* **High-Fidelity Force Sensing**: A 4-cell HX711 cross-configuration matrix decouples physical push/pull forces into pure X, Y, and Z Cartesian vectors, complete with EMA low-pass filtering and auto-tare calibration.

## 🎮 Game Modes

1. **Target Reaching (XY Plane)**: A foundational top-down reaching exercise focusing on planar coordination.
2. **Maze Navigator (XZ Plane)**: A frontal-plane exercise requiring users to navigate a physics-constrained maze, building vertical lifting strength.
3. **Cargo Crane (YZ Plane)**: An isometric-view pickup/drop-off challenge for complex depth and verticality training.

## 🛠️ Hardware Stack

* **Microcontroller**: STM32F407 (programmed via ST HAL)
* **Force Sensors**: 4x HX711 Load Cells (24-bit ADC)
* **Actuators**: 3x Stepper Motors with hardware limit switches for physical homing.
* **Communication**: 115200 baud UART (Custom Telemetry Protocol: `<TGT:x,y,z|K:val|Z:en>`)

## 💻 Software Stack

* **Embedded C**: Manages the 1ms physics interrupts, limit switch debouncing, and hardware constraints.
* **Python (Pygame)**: Renders 60FPS modern, medical-grade UI with dynamic backgrounds, fluid state transitions, and real-time telemetry plotting.

## ⚙️ Physics Implementation

The system deliberately avoids traditional Mass-Spring-Damper impedance models used in collaborative robots to prevent inertia-driven injuries. The physics engine in `main.c` executes:

```c
// First-Order Admittance Control (Virtual Damper)
float speed = (force_toward / CTRL_FORCE_SAT) * CTRL_MAX_SPEED;
```

This ensures the gantry feels like pushing through a thick, stable fluid rather than wielding a heavy, oscillating weight.
