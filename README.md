# FRC 2027 — REBUILT
Team 8533 — Rancho Reapers

# LED Colors
The robot uses an addressable LED system to provide visual feedback during operation. Below is a summary of the LED color codes and their meanings:
| **Color** | **Pattern** | **Active** | **Description**
|------------------|-----|--------------|--------------------|
| ![R](https://img.shields.io/badge/-%20-red?style=flat&logo=&logoColor=)![O](https://img.shields.io/badge/-%20-orange?style=flat&logo=&logoColor=)![Y](https://img.shields.io/badge/-%20-yellow?style=flat&logo=&logoColor=)![G](https://img.shields.io/badge/-%20-green?style=flat&logo=&logoColor=)![B](https://img.shields.io/badge/-%20-blue?style=flat&logo=&logoColor=)![I](https://img.shields.io/badge/-%20-indigo?style=flat&logo=&logoColor=)![V](https://img.shields.io/badge/-%20-violet?style=flat&logo=&logoColor=) Rainbow | Rolling | when in AUTONOMOUS | The robot is in AUTONOMOUS mode. |
| ![1](https://img.shields.io/badge/_-FF9A00?style=for-the-badge) Orange | Solid | when CONNECTED to an FMS and DISABLED | The robot does not yet know what alliance it is on. |
| ![2](https://img.shields.io/badge/_-FF0000?style=for-the-badge) Red | Solid | when CONNECTED to an FMS and DISABLED | The robot is on the RED ALLIANCE. |
| ![3](https://img.shields.io/badge/_-0000FF?style=for-the-badge) Blue | Solid | when CONNECTED to an FMS and DISABLED | The robot is on the BLUE ALLIANCE. |
| ![R](https://img.shields.io/badge/-%20-red?style=flat&logo=&logoColor=)![O](https://img.shields.io/badge/-%20-orange?style=flat&logo=&logoColor=)![Y](https://img.shields.io/badge/-%20-yellow?style=flat&logo=&logoColor=)![G](https://img.shields.io/badge/-%20-green?style=flat&logo=&logoColor=)![B](https://img.shields.io/badge/-%20-blue?style=flat&logo=&logoColor=)![I](https://img.shields.io/badge/-%20-indigo?style=flat&logo=&logoColor=)![V](https://img.shields.io/badge/-%20-violet?style=flat&logo=&logoColor=) Rainbow | Rolling | when CONNECTED to an FMS and in TELEOP | It is the ENDGAME (<30s left in the match). Note that this can be overridden by limelight-related LED patterns. |
| ![4](https://img.shields.io/badge/_-C300FF?style=for-the-badge) Purple | Solid | when NOT CONNECTED to an FMS and DISABLED | The robot is DISABLED. |
| ![5](https://img.shields.io/badge/_-FFFF00?style=for-the-badge) Yellow and ![11](https://img.shields.io/badge/_-C300FF?style=for-the-badge) Purple | Flashing | when NOT CONNECTED to an FMS and in TELEOP | The robot is in TELEOP and ENABLED (flashes matching the RSL). |

## Notes
- AUTO PATHS are relative to BLUE ALLIANCE -> when selecting in match, they will be relative to whichever alliance the robot is on.

## Controls
Rumble Patterns:
- Endgame Notification: Rumble that increases in intensity as endgame comes closer (between 32-29 seconds remaining in match -- endgame begins at 30s). Will rumble on both controllers.

## Other
- WPILib docs: https://docs.wpilib.org
- Use GitHub Issues for feature requests and issue tracking
