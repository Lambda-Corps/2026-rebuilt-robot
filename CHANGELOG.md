# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Support for multiple PhotonVision cameras. Each camera feeds its own pose
  estimate into the drivetrain's pose estimator, and a failure on one camera
  no longer blocks the others.
- `camera_config.py`, which defines each camera's PhotonVision name and
  robot-relative mounting pose (`CameraConfig` / `CAMERAS`).
- Rear-facing second camera (`OV9281_Rear`). Its name and mounting pose are
  placeholders that mirror the front camera and must be updated to match the
  real robot and the PhotonVision UI before deploying.
- Per-camera dashboard values: `Vision/<camera name>/CameraConnected` and
  `Vision/<camera name>/HasTargets`.
- All configured cameras are included in the PhotonVision simulation.
- Vision pose estimates that fall outside the field (beyond
  `VISION_FIELD_MARGIN`) or off the floor (beyond `VISION_MAX_Z_ERROR`) are
  rejected before reaching the drivetrain or seeding the pose.

### Changed

- The vision pipeline reads each camera's PhotonVision pipeline results with
  `getAllUnreadResults()` once per loop and adds a measurement for every new
  result, replacing `getLatestResult()`, which could skip results or add the
  same result twice.
- `has_targets()`, `get_visible_tag_ids()`, `get_best_target_id()`, and
  `seed_drivetrain_pose()` use each camera's newest pipeline result cached by
  `periodic()` instead of decoding the result again on every call. A cached
  result older than `VISION_RESULT_MAX_AGE` counts as "no targets".
- `Vision/CameraConnected` is now true only when every camera is connected.
- `Vision/HasTargets` and `VisionSubsystem.has_targets()` are true when any
  camera sees a tag.
- `VisionSubsystem.get_visible_tag_ids()` combines tags from all cameras
  without duplicates.
- `VisionSubsystem.get_best_target_id()` uses the first camera in `CAMERAS`
  that sees a tag.
- `VisionSubsystem.seed_drivetrain_pose()` seeds from the most confident
  estimate across all cameras.
- `Vision/TimeSinceUpdate` is published every loop instead of only when a
  measurement is accepted.

### Removed

- `CAMERA_NAME` and `ROBOT_TO_CAMERA` from `constants.py`; camera settings
  now live in `camera_config.py`.

## [0.1.0] - 2026-10-06

- Robot code as deployed at competition (`144a0e5`). Changes before this
  changelog was started are recorded in the git history.

[Unreleased]: https://github.com/Lambda-Corps/2026-rebuilt-robot/compare/144a0e5...HEAD
[0.1.0]: https://github.com/Lambda-Corps/2026-rebuilt-robot/tree/144a0e5
