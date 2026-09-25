# Simulation Module — Phase‑0 MicroDrone

The Simulation module contains the Webots environment, 
drone model, and controller scripts used for Phase‑0 testing.

## What This Module Does
- Provides the Webots world + assets
- Spawns the drone model
- Runs controller scripts for motion + CV
- Allows safe testing of behaviors before hardware

## How to Run
Open Webots and load:

simulation/worlds/microdrone_world.wbt

## File Structure
- `worlds/` — Webots world files
- `controllers/` — Python controllers for drone behavior
- `models/` — drone + environment models
- `assets/` — textures, meshes, props

## Good First Issues
- Add a new environment asset
- Improve drone physics parameters
- Add interior‑cart model
- Add a simple test world

## Contributing
- Keep worlds lightweight
- Document controller assumptions
- Test motion engine changes here first
