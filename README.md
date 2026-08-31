
---

# **Phase‑0 Autonomous MicroDrone Wildlife Deterrence System**  
### CONTEXTUAL·AI™ SYSTEMS — RCOS Fall 2026

![CONTEXTUAL·AI SYSTEMS Logo](https://raw.githubusercontent.com/contextualai-systems/Phase0-MicroDrone/refs/heads/main/assets/assets/banner/ContextualAI%20logo.png)



A roof‑mounted autonomous micro‑drone designed to detect wildlife, classify species, execute safe deterrence behaviors, and return to its dock using contextual AI, multi‑axis motion, and strict safety layers.


---

## 📄 Student Engineering Packet (2026)

Complete 51‑page engineering documentation:

PDF Version  
https://github.com/contextualai-systems/Phase0-MicroDrone/blob/main/docs/STUDENT%20ENGINEERING%20PACKET%20V3.pdf (github.com in Bing)

DOCX Version  
https://github.com/contextualai-systems/Phase0-MicroDrone/blob/main/docs/STUDENT%20ENGINEERING%20PACKET%20V3.docx (github.com in Bing)


---

## 🧠 System Overview

The Phase‑0 MicroDrone is built around five core engineering modules:

1. **Computer Vision Pipeline**  
2. **Motion Engine**  
3. **Safety Layer**  
4. **Docking System**  
5. **Simulation Environment**

---

## 🏗️ Project Structure

```
Phase0-MicroDrone/
│
├── assets/                 # QR codes, handouts, presentations
│   ├── qr/                 # QR code images
│   └── presentations/      # RCOS decks + recruitment materials
│
├── docs/                   # Student Engineering Packet (PDF + DOCX)
│
├── cv/                     # Computer vision pipeline
├── motion_engine/          # Multi-axis motion engine
├── safety_layer/           # Safety overrides + constraints
├── docking/                # Docking logic + alignment sensor
│
├── electronics/            # Wiring diagrams + GPIO maps
├── hardware/               # Frame, motors, ESCs, dock components
│
├── simulation/             # Gazebo/Webots simulation configs
├── tests/                  # Unit + integration tests
│
├── src/                    # Main Jetson Nano runtime code
│
├── README.md               # (You are here)
└── LICENSE.md              # MIT License
```

## 2.10 Architecture Summary Diagram
![Phase‑0 Architecture Diagram](assets/architecture/phase0-architecture.png)

## 2.11 Quick‑Start Flow
![Phase‑0 Autonomous Drone Quick‑Start Flow](assets/quickstart/phase0-quickstart.png)

This diagram illustrates the autonomous operational cycle of the Phase‑0 MicroDrone:
1. Launch from roof dock  
2. Autonomous flight and sensing  
3. Return‑to‑dock  
4. Recharge on dock  
5. Ready for next mission  

No cloud compute, no cart control — the drone operates fully autonomously.

---

## 🧩 Getting Started

### Clone the repository
```
git clone https://github.com/contextualai-systems/Phase0-MicroDrone
```

### Install Python dependencies
```
pip install -r requirements.txt
```

### Run the system
```
python3 src/main.py
```

---

## 👥 RCOS Student Teams (Fall 2026)

- Computer Vision  
- Motion Engine  
- Safety Layer  
- Docking System  
- Electronics + Hardware  
- Simulation  

---

## 🛠️ Contributing

- Fork the repo  
- Create a feature branch  
- Submit a pull request  
- Follow coding standards in `/docs`  
- Test safety overrides before flight  

---

## 📬 Contact

**Jeffrey N. Barat**  
Founder — CONTEXTUAL·AI™ SYSTEMS  
RPI RCOS Project Lead  
Palm Beach Gardens, FL

---





