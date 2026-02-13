# Releasing ROS Packages

This repository uses a split policy:

- `ros2/trochoids_ros2`: official ROS apt release target.
- `ros1/trochoids_ros1`: local compatibility package only (no official apt release target).

## ROS2 Release Checklist (Bloom)

1. Ensure CI is green, especially `ROS2 Release-Target Build (Humble)` in `build-all-targets`.
2. Bump package version in `ros2/trochoids_ros2/package.xml`.
3. Tag and push the source repository version.
4. Run bloom release:
   - First release/track:
     - `bloom-release --new-track --rosdistro humble --track humble trochoids_ros2`
   - Subsequent releases:
     - `bloom-release --rosdistro humble trochoids_ros2`
5. Review and merge the generated rosdistro PR.
6. Wait for ROS buildfarm and sync; package becomes available via apt.

## ROS1 Note

`ros1/trochoids_ros1` remains useful for local/noetic workflows and compatibility testing,
but this repository does not target official ROS1 apt publication.
