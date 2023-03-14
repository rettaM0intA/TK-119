// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.enums.CameraMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CycleCameraModeCommand extends InstantCommand {

  /**
   * Changes to the selected vision mode. Use before switching off of auton or to
   * manually switch the mode.
   */
  public CycleCameraModeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CameraMode desiredCameraMode = CameraMode.gamePieceDetector;

    if (RobotContainer.camera.getCurrentCameraMode() == CameraMode.gamePieceDetector) {
      desiredCameraMode = CameraMode.off;
    } else if (RobotContainer.camera.getCurrentCameraMode() == CameraMode.off) {
      desiredCameraMode = CameraMode.aprilTag;
    } else if (RobotContainer.camera.getCurrentCameraMode() == CameraMode.aprilTag) {
      desiredCameraMode = CameraMode.gamePieceDetector;
    }

    // Changes CameraMode based on current CameraMode
    RobotContainer.camera.CameraMode(desiredCameraMode);

  }
}
