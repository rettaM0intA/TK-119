// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OrientWheelsCommand extends CommandBase {
  Timer timer = new Timer();

  /** Creates a new StraightenWheelsCommandReal. */
  public OrientWheelsCommand() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    RobotContainer.driveline.resetSwerveSteerEncoders();
    RobotContainer.driveline.straightenSteerMotors();
    RobotContainer.driveline.drive(0, 0, 0, RobotContainer.driveline.getFieldOrientedModeActive());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // RobotContainer.driveline.orientWheels(xComponent, yComponent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 1) {
      return true;
    }
    return false;
  }
}
