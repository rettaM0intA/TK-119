// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeClawWheelDirectionCommand extends InstantCommand {

  private ClawWheelDirection direction;

  public ChangeClawWheelDirectionCommand(ClawWheelDirection _direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);

    direction = _direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(direction == ClawWheelDirection.in)
      RobotContainer.claw.Spin(.1);
    else if(direction == ClawWheelDirection.out){
      RobotContainer.claw.Spin(-.13);
    }else{
      RobotContainer.claw.Spin(0);
    }
  }
}
