# 2025 REV Swerve Template 

### Malfunctionz - West High School Robotics Club
#### West High School, Knoxville, TN

---

## Description

This repository contains the robot code for a generic REV SparkMax based Swerve Drive using a NavX2-MXP gyroscope.

The project builds on the [REV MAXSwerve Java Template](https://github.com/REVrobotics/MAXSwerve-Java-Template/), designed for an FRC swerve drive using REV MAXSwerve Modules.

I'm not a fan of the Command-Based programming style.  Passing functions as objects doesn't sit well in my brain. So I rewrote the template to drop the command-based functionality.  

I'm attempting to remove as much as possible from this code while maintaining the basic functionality of a swerve drive.
---

## Getting Started

### Prerequisites

Install the 2025 WPILib VS Code environment:
1. Follow the instructions [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
2. Download `WPILib_Windows-2025.2.1.ISO`.
3. Right-click the ISO file and select "Mount" as a disk.
4. Run `D:\WPILibInstaller.exe`.
5. Install for all users and download for this computer only.
6. After completing the installation, eject the ISO by right-clicking the `D:` drive and selecting "Eject."

### Setting Up the Project

1. Open 2025 WPILib VS Code.
2. Fork a copy of the main repository in GitHub
   - Go to https://github.com/rrmcmurry/2025RevSwerveTemplate
   - Click on the button that says Fork in the upper right corner
   - This should make a copy of the repository in your own GitHub account.
3. Clone your forked GitHub repository locally in VS Code:
   - On the Welcome Tab under "Start" look for "Clone Git Repository" and select it
   - https://github.com/YOURUSERNAME/2025RevSwerveTemplate.git <- Type this in but use your github username   
4. Verify the setup:
   - Check for "Build Successful" in the Terminal.
5. Navigate to the project files:
   - `2025RevSwerveTemplate/src/main/java/frc/robot`
   - Focus on modifying `Robot.java`, `Constants.java`, and any files under `subsystems`.
6. Create a branch
   - If you plan to make changes and want those changes merged back into the original project, I recommend you make a branch with your name on it.
   - Make changes in that branch



