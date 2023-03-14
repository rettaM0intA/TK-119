// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TestRotationalDriveCommand extends CommandBase {

  Timer timer = new Timer();
  double inches = 0;
  double angle = 0;
  double driveSpeed = 0;
  double goalAngle = 0;
  double rotationalSpeed = 0;
  double maxRotationalSpeed = 0;
  double time = 0;
  boolean fieldOriented = false;

  /** Creates a new TestRotationalDriveCommand. */
  /**
   * 
   * @param _inches          how far the robot will move (inches)
   * @param _angle           the angle at which the robot will move (degrees)
   * @param _driveSpeed      the drive speed of the robot
   * @param _goalAngle       the angle to which the robot will turn (degrees)
   * @param _rotationalSpeed the turn speed of the robot
   * @param _time            the maximum duration of the command
   * @param _fieldOriented   whether the robot is in field-oriented mode
   */
  public TestRotationalDriveCommand(double _inches, double _angle, double _driveSpeed, double _goalAngle,
      double _rotationalSpeed, double _time,
      boolean _fieldOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);

    inches = _inches;
    angle = _angle;
    driveSpeed = _driveSpeed;
    goalAngle = _goalAngle;
    rotationalSpeed = _rotationalSpeed;
    time = _time;
    fieldOriented = _fieldOriented;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveline.resetGyro();
    RobotContainer.driveline.drive(0, 0, 0, true);
    RobotContainer.driveline.resetSwerveSteerEncoders();
    RobotContainer.driveline.resetSwerveDriveEncoders();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.driveline.getRobotAngle() - goalAngle) > 1)
      rotationalSpeed = (RobotContainer.driveline.getRobotAngle() - goalAngle) * .05;
    if (rotationalSpeed > 0.3)
      rotationalSpeed = 0.3;
    else if (rotationalSpeed < -0.3)
      rotationalSpeed = -0.3;
    else {
      rotationalSpeed = 0;
    }

    // if (Math.abs(RobotContainer.driveline.getRobotAngle() - goalAngle) > 5) {
      // RobotContainer.driveline.drive(Math.sin(angle * Math.PI / 180) * driveSpeed,
      // Math.cos(angle * Math.PI / 180) * driveSpeed, rotationalSpeed, fieldOriented);
    // } else {
    //   RobotContainer.driveline.drive(rotationalSpeed, driveSpeed, 0, fieldOriented);
    // }

    // if (Math.abs(RobotContainer.driveline.getRobotAngle() - goalAngle) < 2) {
    //   RobotContainer.driveline.resetSwerveDriveEncoders();
    // }

    RobotContainer.driveline.drive(Math.sin(angle * Math.PI / 180) * driveSpeed,
        Math.cos(angle * Math.PI / 180) * driveSpeed, rotationalSpeed, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveline.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean rtn = false;

    // Going to the left needs to return a positive angle
    // When the drive() method is given a negative input for _rot, the robot turns
    // to the left

    // if (goalAngle > 0) {
    // if (RobotContainer.driveline.getAverageDistanceInInches() >= inches
    // && RobotContainer.driveline.getRobotAngle() >= angle) {
    // rtn = true;
    // }
    // } else if (goalAngle < 0) {
    // if (RobotContainer.driveline.getAverageDistanceInInches() >= inches
    // && RobotContainer.driveline.getRobotAngle() <= angle) {
    // rtn = true;
    // }
    // } else if (goalAngle == 0) {
    // if (RobotContainer.driveline.getAverageDistanceInInches() >= inches) {
    // rtn = true;
    // }
    // }

    if ((RobotContainer.driveline.getAverageDistanceInInches() >= inches &&
    ((goalAngle > 0 && RobotContainer.driveline.getRobotAngle() >= goalAngle) ||
    (goalAngle < 0 && RobotContainer.driveline.getRobotAngle() <= goalAngle))) ||
    timer.get() >= time) {
    rtn = true;
    }

    if (goalAngle == 0) {
      if (RobotContainer.driveline.getAverageDistanceInInches() >= inches) {
        rtn = true;
      }
    }

    return rtn;
  }
}
