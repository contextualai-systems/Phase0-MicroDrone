# Docking Module — Phase‑0 MicroDrone

The Docking module handles autonomous docking, alignment, 
and charging logic for the roof‑mounted docking station.

## What This Module Does
- Detects docking station alignment
- Controls approach + descent
- Manages charging contact logic
- Interfaces with motion engine for fine adjustments

## How to Run
```bash
python docking/run_docking.py
File Structure
alignment/ — visual + sensor alignment logic

approach/ — descent + positioning

charging/ — contact detection + state machine

utils/ — geometry + smoothing

Good First Issues
Add alignment visualization

Improve descent stability

Add charging‑state logging

Add mock docking station for simulation

Contributing
Document alignment assumptions

Test docking in simulation before merging

Add diagrams for approach logic
