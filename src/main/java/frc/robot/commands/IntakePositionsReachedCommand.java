// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakePositionsReachedCommand extends CommandBase {

  int maxTime;
  Timer timer = new Timer();

  /**
   * Creates a new IntakePositionsReachedCommand.
   * 
   * @param _maxTime the amount of seconds
   */
  public IntakePositionsReachedCommand(int _maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.

    maxTime = _maxTime;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putBoolean("Elevator goal reached",
    // RobotContainer.elevator.goalReached());
    // SmartDashboard.putBoolean("Extender goal reached",
    // RobotContainer.extender.goalReached());
    // SmartDashboard.putBoolean("Hinge goal reached",
    // RobotContainer.hinge.goalReached());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > maxTime ||
        (RobotContainer.elevator.goalReached() &&
            RobotContainer.extender.goalReached() &&
            RobotContainer.hinge.goalReached())) {
      return true;
    }
    return false;
  }
}
