// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.enums.CameraMode;

public class VisionAssistedGamepieceLocator extends CommandBase {

  int rightDistanceCounter = 0;
  double goalRotation = 0;
  boolean shouldRotate = false;

  /** Creates a new VisionAssistedItemPlacement. */
  public VisionAssistedGamepieceLocator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    addRequirements(RobotContainer.camera);

    rightDistanceCounter = 0;

    this.goalRotation = 0;
    shouldRotate = false;
  }

  /**
   * Creates a new VisionAssistedItemPlacement.
   * 
   * @param goalRotation the goal rotation of the auton.
   */
  public VisionAssistedGamepieceLocator(double goalRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    addRequirements(RobotContainer.camera);

    RobotContainer.camera.CameraMode(CameraMode.gamePieceDetector);

    rightDistanceCounter = 0;

    this.goalRotation = goalRotation;
    shouldRotate = false; // Originally set to true
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightDistanceCounter = 0;

    RobotContainer.camera.CameraMode(CameraMode.gamePieceDetector);

    RobotContainer.driveline.straightenSteerMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double horizontal = 0;
    double rotate = 0;

    horizontal = (RobotContainer.camera.tx.getDouble(0) + 4) * .02;

    if (horizontal > .05)
      horizontal = .05;
    else if (horizontal < -.05)
      horizontal = -.05;

    if (shouldRotate) {
      if (Math.abs(RobotContainer.driveline.getRobotAngle() - goalRotation) > 2)
        rotate = (RobotContainer.driveline.getRobotAngle() - goalRotation) * .05;

      if (rotate > .15)
        rotate = .15;
      else if (rotate < -.15)
        rotate = -.15;
    } else {
      rotate = 0;
    }

    if (RobotContainer.camera.TargetLocated() &&
        RobotContainer.camera.tx.getDouble(-500) > -4 &&
        RobotContainer.camera.tx.getDouble(500) < -2 &&
        (!shouldRotate || (shouldRotate &&
            Math.abs(RobotContainer.driveline.getRobotAngle() - goalRotation) < 2))) {

      RobotContainer.driveline.drive(0, 0, 0, false);
      rightDistanceCounter += 1;

    } else if (RobotContainer.camera.TargetLocated()) {

      if (shouldRotate && (RobotContainer.driveline.getRobotAngle() - goalRotation) < 8)
        RobotContainer.driveline.drive(horizontal, 0, rotate, false);
      else
        RobotContainer.driveline.drive(horizontal, 0, rotate, false);
      // RobotContainer.driveline.drive(0, 0, rotate, false);

    } else {
      RobotContainer.driveline.drive(horizontal, 0, rotate, false);
      // RobotContainer.driveline.drive(0, 0, rotate, false);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(RobotContainer.camera.TargetLocated() &&
    // RobotContainer.camera.tx.getDouble(0) > -1 &&
    // RobotContainer.camera.tx.getDouble(0) < 1){
    // RobotContainer.driveline.drive(0, 0, 0, false);
    // return true;
    // }

    if (rightDistanceCounter >= 4) {
      RobotContainer.driveline.drive(0, 0, 0, false);
      return true;
    }
    return false;
  }
}
