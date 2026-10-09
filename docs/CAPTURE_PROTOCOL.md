# Capture protocol (stock capture route)

## Route selected
Stock iPhone camera capture with pipeline-defined folder handoff. No third-party scanner app is claimed or required. This route supplies photo stills; video uses the native Camera app. LiDAR files require an explicit export bundle.

## Photo tier
1. On iPhone 15 or newer, open Camera and choose Photo. Set capture format to a mode producing an OpenCV-readable image (JPEG recommended), then use the main 1x camera, standard resolution, flash off unless the room is otherwise too dark; if flash is needed, flag the room as low-light/flash-affected.
2. For each room take 2-8 stills from distinct positions, with overlap between adjacent views. Include each wall and the floor/ceiling junction where safe. Avoid digital zoom, motion blur, mirrors, glass reflections, wet-look surfaces, and moving people. Do not enter unsafe areas.
3. At transitions, take a view that shows the connecting doorway from each side. Assign room IDs explicitly. Do not infer room identity from filenames.
4. Export original files without edits, create one folder per room, and add `capture.json` from the template. Copy the folder to the machine running the pipeline.
5. Run `python -m src.pipeline --input <capture-folder> --output <new-output-folder>`.

## Video tier
Record a standard handheld walkthrough with the native Camera app using a codec that the local OpenCV build can decode; codec compatibility is machine-dependent and not yet validated. Walk slowly at a steady height, pause briefly at each room entrance and turn, keep continuous overlap, and avoid fast pans/zoom. Export the original clip and reference it in `capture.json`.

## LiDAR tier
Supply depth, poses, intrinsics, plus explicit units, quaternion order, pose convention, and RGB-depth registration in the room manifest. Unknown conventions are rejected. For frame folders with odometry CSV, start from `capture_template/lidar_sequence.capture.json` and replace every placeholder with metadata confirmed by the capture tool/device record.

No vendor stock scanning application/version is claimed: this is a capture protocol using installed iOS Camera functionality. The pipeline extracts adjacent-view feature matches and relative camera pose from stills. It does not yet infer a metric footprint or dimensions from these features. The current pipeline also inventories and hashes capture files.

## LiDAR bundle formats
The pipeline accepts either a single JSON depth frame with a JSON pose list and JSON intrinsics, or a PNG depth sequence with an odometry CSV. For the sequence format, set `depth_dir`, `odometry_csv`, `depth_unit`, `pose_unit`, `quaternion_order`, `pose_convention`, `coordinate_handedness`, `world_up`, and `intrinsics_grid` in the room manifest. The CSV must have timestamp, frame, translation, quaternion, and per-frame `fx`, `fy`, `cx`, `cy` columns. Frame filenames are joined to the CSV `frame` column by exact stem. RGB-grid intrinsics also require an explicit `rgb_depth_registration` mapping. `confidence_dir` is optional; provide a documented `confidence_min` to apply confidence filtering. The pipeline samples frames evenly across the sequence and records their IDs. Units and conventions still need authoritative capture metadata, and output remains provisional until calibration and ground-truth checks pass. The supplied folders can be inventoried without those declarations, but should not be treated as calibrated measurements.
