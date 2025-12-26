# Maze Solving Robot – Flood Fill Algorithm

## 📌 Project Overview
This repository contains the design and partial implementation of a **maze-solving mobile robot** using the **Flood Fill algorithm**, developed as part of a **course project for Robotics Implementation Methods**.

The project focuses on:
- Algorithm development and validation in MATLAB
- Logic-level conversion to C++ for real robot deployment
- Embedded-system–friendly design suitable for Arduino-based robots

---

## 🧠 Algorithm Description
The robot uses a **Flood Fill (Breadth-First Search)** approach to navigate a grid-based maze:

1. The robot starts with knowledge of **only the outer boundary walls**
2. As it moves, it **senses walls** using onboard sensors
3. The internal maze map is updated dynamically
4. A flood-fill algorithm computes distances from the goal
5. The robot greedily moves to the neighboring cell with the lowest distance
6. The process repeats until the goal is reached

This approach is commonly used in **Micromouse and autonomous maze-solving robots**.


---

## 🧪 MATLAB Implementation
The MATLAB code:
- Simulates a **6×6 maze**
- Includes dynamic wall discovery
- Uses flood fill for path planning
- Provides visualization for debugging and validation

📌 **Purpose**: Algorithm verification and conceptual validation  
📌 **Note**: MATLAB visualization is **not intended for real robot execution**

---

## 🤖 Arduino / C++ Implementation
The Arduino code:
- Is written in **Arduino C++**
- Implements the same flood-fill logic as MATLAB
- Uses fixed-size arrays (no dynamic memory or STL)
- Is suitable for deployment on microcontrollers
- Excludes all visualization and simulation components

📌 **Purpose**: Logic-level implementation for a real robot platform  
📌 **Hardware assumed**:
- Differential drive robot
- IR or ToF wall sensors
- Arduino-compatible microcontroller

---

## ⏸️ Project Status
🚧 **Development Halted**

Further development of this project has been **intentionally halted**.

The repository represents the **completed course project submission**, and no additional features such as:
- PID tuning
- Encoder-based motion control
- Fast-run optimization
- Hardware-specific calibration
- were implemented beyond this stage.

---

## 🎓 Academic Context
- **Course**: Robotics Implementation Methods  
- **Project Type**: Course Project  
- **Focus**: Algorithm design, simulation, and embedded implementation logic  
- **Outcome**: Successful demonstration of maze-solving strategy and software architecture.

---
