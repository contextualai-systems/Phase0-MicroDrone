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

