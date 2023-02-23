// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDriveStraight extends CommandBase {

  double xSpeed = 0;
  double ySpeed = 0;
  double distanceIn = 0;

  /** Creates a new AutoDriveStraight. */
  public AutoDriveStraight(double x, double y, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);

    xSpeed = x;
    ySpeed = y;
    distanceIn = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Was not working, change to drive
    //RobotContainer.driveline.resetSwerveSteerEncoders();

    //Fix - hopefully
    RobotContainer.driveline.resetSwerveDriveEncoders();


    RobotContainer.driveline.drive(0, 0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveline.drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.driveline.getAverageDistanceInInches() >= distanceIn){
      RobotContainer.driveline.drive(0, 0, 0, false);

      //Reset serve drive encoders
      RobotContainer.driveline.resetSwerveDriveEncoders();
      return true;
    }
    return false;
  }
}
