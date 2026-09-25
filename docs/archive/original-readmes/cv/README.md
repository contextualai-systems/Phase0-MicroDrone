# Computer Vision Module — Phase‑0 MicroDrone

## Purpose
The CV module handles all computer‑vision tasks for Phase‑0.  
In this semester, the focus is on synthetic camera input, preprocessing, and logging — not real hardware.

## Responsibilities (Phase‑0)
- Provide a synthetic camera frame source (static image or test pattern)
- Apply basic preprocessing (resize, grayscale, normalization)
- Log frames for debugging
- Output a simple “detection stub” to feed the autonomy pipeline

## Inputs
- Synthetic camera frame (generated internally)
- Optional test images from assets/

## Outputs
- Preprocessed frame
- Detection stub (placeholder bounding box + label)
- Frame logs for debugging

## File Structure
- run_cv.py — main entry point for CV
- pipeline/ — preprocessing steps
- detectors/ — placeholder detection logic
- utils/ — logging helpers

## Good First Issues
- Add a new preprocessing filter
- Improve bounding box stability in the stub
- Add frame logging
- Add mock camera input for simulation

  ## Future Phases
- Real CSI camera input (Phase‑1)
- Species classification (Phase‑2)
- Approach‑angle estimation (Phase‑2)
- Full object detection pipeline (Phase‑3)



