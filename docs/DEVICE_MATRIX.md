# Device matrix

| Tier | Hardware | Capture | Expected accuracy | Uncertainty | Limitations / calibration |
|---|---|---|---|---|---|
| Photo | iPhone 15 or newer smartphone; model recorded in manifest | 2-8 original stills per room | NOT ESTABLISHED | No interval until empirically calibrated | No depth/poses; OpenCV adjacent-image feature matching estimates relative pose direction using a generic focal/principal-point prior; no EXIF calibration is read, so scale and room geometry are unestablished; calibration requires independent fit and holdout tape/laser measurements. |
| Video | iPhone 15 or newer smartphone | Native handheld walkthrough video | NOT ESTABLISHED | No interval until empirically calibrated | OpenCV selects up to eight frames and estimates adjacent relative motion with generic camera prior; no metric geometry; blur, low light, reflective surfaces degrade evidence. |
| LiDAR | Pro-class device providing depth + poses + intrinsics | Exported sensor bundle with explicit conventions | NOT ESTABLISHED | No interval until empirically calibrated | Loader requires declared units, quaternion order, pose frame, timestamps, handedness, world-up, intrinsics grid, and registration; JSON depth-matrix decoding, single-frame point projection, and deterministic RANSAC surface candidates exist; standard binary/PNG exports, synchronized multi-frame fusion, and room geometry remain unimplemented. |

No cross-tier accuracy equivalence is claimed.
