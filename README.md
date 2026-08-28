# **ContextualAI Systems — Phase‑0 Autonomous MicroDrone**  
### **RCOS Fall 2026 — Autonomous Wildlife Deterrence Drone**

Welcome to the Phase‑0 Autonomous MicroDrone Project, an RPI RCOS initiative to design, build, and test a real autonomous drone system capable of detecting wildlife, making decisions, and executing safe deterrence behaviors.  
This is a hands‑on engineering + autonomy + robotics project.  
You will build the actual drone prototype this semester.

## 📄 One‑Page Project Overview (with QR Code)

Download the one‑page Phase‑0 MicroDrone handout:

- [One‑Page Handout (PDF)](assets/1%20page.pdf)
- [One‑Page Handout (Word)](assets/1%20page.docx)

This handout includes the QR code linking directly to this repository and provides a concise overview of the engineering + autonomy + entrepreneurship goals of the project.
![QR Code](assets/qr/QR%20Code%20ContextualAI%20Systems.png)

---

# **Table of Contents**
1. [Project Overview](#project-overview)  
2. [System Architecture](#system-architecture)  
3. [START HERE — Onboarding](#start-here--phase-0-microdrone-onboarding)  
4. [Tech Stack](#tech-stack)  
5. [Student Teams](#student-teams-choose-your-track)  
6. [Getting Started](#getting-started)  
7. [Good First Issues](#good-first-issues-beginner-friendly)  
8. [Semester Roadmap](#semester-roadmap-fall-2026)  
9. [Contribution Guidelines](#contribution-guidelines)  
10. [Project Resources](#project-resources)  
11. [Recruitment Handout](#recruitment-handout-pdf)  
12. [Contact](#contact)  
13. [QR Code](#qr-code)  
14. [Final Note](#final-note)

---

# **Project Overview**
The Phase‑0 MicroDrone is a roof‑mounted autonomous deterrence drone designed to:

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

# **System Architecture**
```
+-------------------------+
| Sensors                |
| - Front CSI Camera     |
| - Rear Camera          |
| - Interior Sensor      |
| - IMU / IR             |
+-----------+-------------+
            v
+-------------------------+
| Jetson Nano Compute     |
| - Python                |
| - OpenCV                |
| - CV Pipeline           |
+-----------+-------------+
            v
+-------------------------+
| Decision Layer          |
| - Species Logic         |
| - Safety Rules          |
| - Motion Selection      |
+-----------+-------------+
            v
+-------------------------+
| Motion Engine           |
| - Rise/Tilt/Rotate      |
| - Interior Micro-Motion |
+-----------+-------------+
            v
+-------------------------+
| Docking System          |
| - Alignment             |
| - Charging Contacts     |
+-------------------------+
```

---

# **START HERE — Phase‑0 MicroDrone Onboarding**

Welcome to **CONTEXTUAL·AI™ SYSTEMS — Phase‑0 MicroDrone**.  
This section gives you everything you need to get set up, run the project, and begin contributing.

### **1. Install Required Tools**
Install:

- Python 3.10+  
- pip  
- Git  
- VS Code  
- Webots R2024b  
- OpenCV (auto‑installed via requirements)

### **2. Clone the Repository**
```bash
git clone https://github.com/ContextualAI-Systems/Phase0-MicroDrone.git
cd Phase0-MicroDrone
```

### **3. Install Dependencies**
```bash
pip install -r requirements.txt
```

### **4. Project Structure Overview**
```
/cv             # Computer vision pipeline
/motion_engine  # Motion primitives + control logic
/safety_layer   # Collision avoidance + failsafes
/docking        # Autonomous docking + charging
/simulation     # Webots simulation environment
/docs           # Documentation + handouts
/assets         # QR codes + diagrams + branding
```

### **5. Run the Simulation**
Open:
```
simulation/worlds/microdrone_world.wbt
```
Press **Play**.

### **6. Run the CV Pipeline**
```bash
python cv/run_cv.py
```

### **7. Run the Motion Engine**
```bash
python motion_engine/run_motion.py
```

### **8. Good First Issues**
- Add CV filters  
- Improve bounding box stability  
- Add motion primitives  
- Add safety checks  
- Improve simulation realism  

### **9. Contribution Workflow**
1. Create a branch  
2. Make changes  
3. Run tests  
4. Submit PR  
5. Tag team lead  

### **10. Documentation & Resources**
- Recruitment Handout (PDF)  
- Architecture Diagram  
- RCOS Project Page (coming soon)

---

# **Tech Stack**
### **Languages & Frameworks**
- Python  
- OpenCV  
- ROS2 (optional)  
- Gazebo/Webots  
- Embedded C/C++ (optional)

### **Hardware**
- Jetson Nano  
- Brushless motors + ESCs  
- CSI cameras  
- IMU + IR sensors  
- Custom docking system

---

# **Student Teams (Choose Your Track)**

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

# **Getting Started**

### **1. Clone the repository**
```bash
git clone https://github.com/ContextualAI-Systems/Phase0-MicroDrone.git
cd Phase0-MicroDrone
```

### **Quick Access (QR Code)**
Scan the QR code below to open this repository instantly:

`[Looks like the result wasn't safe to show. Let's switch things up and try something else!]`

### **2. Install dependencies**
(Requirements file or Docker container will be provided.)

### **3. Run the simulation**
Instructions will be provided for Gazebo/Webots.

### **4. Pick an issue**
Start with a **Good First Issue**.

---

# **Good First Issues (Beginner Friendly)**

- Add logging to CV pipeline  
- Write documentation for motion engine  
- Create simple OpenCV script  
- Build tilt/rotation behavior in simulation  
- Add interior‑sensor mock data  
- Improve simulation assets  
- Create unit tests for safety rules  

---

# **Semester Roadmap (Fall 2026)**

### **Week 1–2**
Jetson setup, CV pipeline scaffolding, simulation environment

### **Week 3–4**
Motion engine prototype, sensor integration

### **Week 5–6**
Safety layer implementation, interior mode logic

### **Week 7–8**
Docking system integration, full system assembly

### **Week 9–10**
Field testing, refinement, demo preparation

---

# **Contribution Guidelines**
- Open‑source under RCOS guidelines  
- Use branches + pull requests  
- Document your code  
- Write tests where possible  
- Attend weekly stand‑ups  
- Collaborate across sub‑teams  
- Be kind, supportive, and curious  

---

# **Project Resources**
- Student Engineering Packet (PDF)  
- Slides (RCOS Pitch Deck)  
- Slack/Discord (QR code)

---

# **Recruitment Handout (PDF)**
`[Looks like the result wasn't safe to show. Let's switch things up and try something else!]`

---

# **Contact**
**Jeffrey Barat**  
Founder, ContextualAI Systems  
Palm Beach Gardens, FL  
Email: contextualaisystems@yourdomain.com

---
---

# **Final Note**
This is a **real autonomous system**.  
You will build the **actual drone prototype** this semester.  
Your work will be part of a continuing project with opportunities for follow‑on development outside RCOS.

Let’s build something real.




