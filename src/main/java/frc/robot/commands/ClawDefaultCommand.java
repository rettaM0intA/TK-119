// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawDefaultCommand extends CommandBase {
  /** Creates a new ClawDefaultCommand. */
  public ClawDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.operator.getRightTriggerAxis() > .5){
      RobotContainer.claw.Spin(.1);
    }else if(RobotContainer.operator.getLeftTriggerAxis() > .5){
      RobotContainer.claw.Spin(-.13);
    }else{
      RobotContainer.claw.Hold();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
