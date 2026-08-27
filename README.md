# ContextualAI Systems — Phase‑0 Autonomous MicroDrone  
### RCOS Fall 2026 — Autonomous Wildlife Deterrence Drone

Welcome to the **Phase‑0 Autonomous MicroDrone Project**, an RPI RCOS initiative to design, build, and test a real autonomous drone system capable of detecting wildlife, making decisions, and executing safe deterrence behaviors.

This is a **hands‑on engineering + autonomy + robotics project**.  
You will build the **actual drone prototype** this semester.

---

## Project Overview

The Phase‑0 MicroDrone is a **roof‑mounted autonomous deterrence drone** designed to:

- Detect approaching birds  
- Classify species  
- Select deterrence behaviors  
- Execute multi‑axis motion  
- Operate safely around humans  
- Dock autonomously  

The system uses:

- Jetson Nano onboard AI  
- Computer vision  
- Motion engine  
- Safety layer  
- Docking system  
- Simulation environment (Gazebo/Webots)

This project is fully student‑built under RCOS.

---

## System Architecture

```
+-------------------------+
|      Sensors            |
|  - Front CSI Camera     |
|  - Rear Camera          |
|  - Interior Sensor      |
|  - IMU / IR             |
+-----------+-------------+
            |
            v
+-------------------------+
|   Jetson Nano Compute   |
|   - Python              |
|   - OpenCV              |
|   - CV Pipeline         |
+-----------+-------------+
            |
            v
+-------------------------+
|   Decision Layer        |
|   - Species Logic       |
|   - Safety Rules        |
|   - Motion Selection    |
+-----------+-------------+
            |
            v
+-------------------------+
|   Motion Engine         |
|   - Rise/Tilt/Rotate    |
|   - Interior Micro-Motion|
+-----------+-------------+
            |
            v
+-------------------------+
|   Docking System        |
|   - Alignment           |
|   - Charging Contacts   |
+-------------------------+
```
---

## **START HERE — Phase‑0 MicroDrone Onboarding**

Welcome to **CONTEXTUAL·AI™ SYSTEMS — Phase‑0 MicroDrone**.  
This section gives you everything you need to get set up, run the project, and begin contributing.

---

### **1. Install Required Tools**

Before cloning the repo, install the following:

- **Python 3.10+**  
- **pip** (Python package manager)  
- **Git**  
- **VS Code** (recommended)  
- **Webots R2024b** (simulation environment)  
- **OpenCV** (installed automatically via requirements)

---

### **2. Clone the Repository**

```bash
git clone https://github.com/ContextualAI-Systems/Phase0-MicroDrone.git
cd Phase0-MicroDrone
```

If you prefer quick access, scan the QR code in the section above.

---

### **3. Install Dependencies**

```bash
pip install -r requirements.txt
```

This installs:

- OpenCV  
- NumPy  
- PyTorch (CPU version)  
- Webots Python API  
- Utility libraries used across modules  

---

### **4. Project Structure Overview**

This repository is organized into the core Phase‑0 modules:

```
/cv                     # Computer vision pipeline (object detection, tracking)
/motion_engine          # Drone motion primitives and control logic
/safety_layer           # Collision avoidance, failsafes, boundary enforcement
/docking                # Autonomous docking + charging logic
/simulation             # Webots simulation environment and drone model
/docs                   # Project documentation, handouts, and onboarding guides
/assets                 # QR codes, diagrams, and branding assets
```

Each module contains its own README with instructions and starter tasks.

---

### **5. Run the Simulation (Webots)**

1. Install **Webots R2024b**  
2. Open the simulation world:

```
simulation/worlds/microdrone_world.wbt
```

3. Press **Play**  
4. The drone will spawn with default Phase‑0 behaviors

---

### **6. Run the CV Pipeline**

From the project root:

```bash
python cv/run_cv.py
```

This launches:

- camera input (simulated or real)  
- object detection  
- bounding box visualization  
- frame‑by‑frame logging  

---

### **7. Run the Motion Engine**

```bash
python motion_engine/run_motion.py
```

This executes:

- basic motion primitives  
- hover, rotate, translate  
- safety‑checked movement commands  

---

### **8. Good First Issues**

If you’re new to the project, start here:

- Add new CV filters  
- Improve bounding box stability  
- Add new motion primitives  
- Add safety checks for edge cases  
- Improve simulation realism  

All beginner‑friendly tasks are labeled **Good First Issue** in GitHub.

---

### **9. Contribution Workflow**

1. Create a branch  
2. Make your changes  
3. Run tests  
4. Submit a Pull Request  
5. Tag your team lead for review  

Full details are in **CONTRIBUTING.md**.

---

### **10. Documentation & Resources**

- **Recruitment Handout (PDF)** — in `/docs`  
- **System Architecture Diagram** — in `/assets`  
- **RCOS Project Page** — coming soon  

---

## Tech Stack

### Languages & Frameworks
- Python  
- OpenCV  
- ROS2 (optional)  
- Gazebo or Webots simulation  
- Embedded C/C++ (optional)

### Hardware
- Jetson Nano  
- Brushless motors + ESCs  
- CSI cameras  
- IMU + IR sensors  
- Custom docking system

---

## Student Teams (Choose Your Track)

### **Computer Vision Team**
- Species detection  
- Approach‑angle detection  
- Interior‑cart detection  
- OpenCV + Python

### **Motion Engine Team**
- Rise / tilt / rotation logic  
- Interior micro‑motion  
- Control algorithms

### **Safety Layer Team**
- Human detection  
- No‑launch zones  
- Interior mode logic  
- Rule‑based systems

### **Docking & Electronics Team**
- Dock sensor logic  
- Charging contacts  
- Alignment system  
- GPIO + embedded systems

### **Simulation Team**
- Gazebo/Webots environment  
- Drone physics  
- Motion engine testing

---

## Getting Started

### 1. Clone the repository
```
git clone https://github.com/ContextualAI-Systems/Phase0-MicroDrone.git
cd Phase0-MicroDrone
## Quick Access (QR Code)

Scan the QR code below to open this repository instantly:

![QR Code ContextualAI Systems.png](assets/qr/your-qrcode-file.png)


### 2. Install dependencies
(Requirements file or Docker container will be provided.)

### 3. Run the simulation
Instructions will be provided for Gazebo/Webots.

### 4. Pick an issue  
Start with a **Good First Issue** (see below).

---

## Good First Issues (Beginner Friendly)

- Add logging to CV pipeline  
- Write documentation for motion engine  
- Create simple OpenCV script (thresholding, contour detection)  
- Build basic tilt/rotation behavior in simulation  
- Add interior‑sensor mock data  
- Improve simulation environment assets  
- Create unit tests for safety rules  

These are designed so **freshmen and sophomores can contribute immediately**.

---

## Semester Roadmap (Fall 2026)

### Week 1–2  
Jetson setup, CV pipeline scaffolding, simulation environment

### Week 3–4  
Motion engine prototype, sensor integration

### Week 5–6  
Safety layer implementation, interior mode logic

### Week 7–8  
Docking system integration, full system assembly

### Week 9–10  
Field testing, refinement, demo preparation

---

## Contribution Guidelines

- All contributions must be open‑source under RCOS guidelines  
- Use branches + pull requests  
- Document your code  
- Write tests where possible  
- Attend weekly stand‑ups  
- Collaborate across sub‑teams  
- Be kind, supportive, and curious

---

## Project Resources

- **Student Engineering Packet (PDF)**  
- **Slides (RCOS Pitch Deck)**  
- **Slack/Discord (QR code)**  
### Recruitment Handout (PDF)
[2026 RPI Recruitment Handout](docs/2026%20RPI%20Recruitment%20Handoutv6.pdf)

---

## Contact

**Jeffrey Barat**  
Founder, ContextualAI Systems  
Palm Beach Gardens, FL  
Email: contextualaisystems@yourdomain.com

---

## QR Code   
QR Code ContextualAI Systems.png


---

## Final Note

This is a **real autonomous system**.  
You will build the **actual drone prototype** this semester.  
Your work will be part of a continuing project with opportunities for follow‑on development outside RCOS.


Let’s build something real.
