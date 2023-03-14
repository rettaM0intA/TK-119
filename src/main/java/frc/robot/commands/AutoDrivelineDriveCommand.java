// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDrivelineDriveCommand extends CommandBase {

  double xSpeed = 0;
  double ySpeed = 0;
  double distanceIn = 0;
  boolean fieldOrientedMode;

  /** Creates a new AutoDriveStraight. */
  /**
   * This command is obsolete.
   */
  public AutoDrivelineDriveCommand(double _x, double _y, double _distance, boolean _fieldOrientedMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);

    xSpeed = _x;
    ySpeed = _y;
    distanceIn = _distance;
    fieldOrientedMode = _fieldOrientedMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveline.resetSwerveDriveEncoders();
    
    RobotContainer.driveline.drive(0, 0, 0, fieldOrientedMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveline.drive(xSpeed, ySpeed, 0, fieldOrientedMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveline.drive(0, 0, 0, fieldOrientedMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.driveline.getAverageDistanceInInches() >= distanceIn) {
      RobotContainer.driveline.drive(0, 0, 0, fieldOrientedMode);

      RobotContainer.driveline.resetSwerveDriveEncoders();
      return true;
    }
    return false;
  }
}
