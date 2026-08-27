# CV Module — Phase‑0 MicroDrone

The CV module handles all computer vision tasks for the Phase‑0 MicroDrone. 
This includes object detection, species classification, approach‑angle estimation, 
and interior‑cart detection.

## What This Module Does
- Reads frames from the front CSI camera (or simulation feed)
- Runs object detection (birds, carts, humans)
- Computes bounding boxes + confidence scores
- Logs frames for debugging
- Sends detection results to the Decision Layer

## How to Run
```bash
python cv/run_cv.py

## File Structure
- `run_cv.py` — main entry point
- `pipeline/` — preprocessing, filtering, transforms
- `detectors/` — species + object detection models
- `utils/` — logging, frame helpers

## Good First Issues
- Add a new OpenCV filter
- Improve bounding box stability
- Add frame logging
- Add mock camera input for simulation

## Contributing
- Keep functions small and testable
- Document new detectors
- Add sample images for debugging
