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

![GitHub QR Code](assets/qr/your-qrcode-file.png)


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

---

## Contact

**Jeffrey Barat**  
Founder, ContextualAI Systems  
Palm Beach Gardens, FL  
Email: contextualaisystems@yourdomain.com

---

## QR Code Placeholder  
(Insert QR code linking to Slack/Discord + GitHub repo)

---

## Final Note

This is a **real autonomous system**.  
You will build the **actual drone prototype** this semester.  
Your work will be part of a continuing project with opportunities for follow‑on development outside RCOS.


Let’s build something real.
