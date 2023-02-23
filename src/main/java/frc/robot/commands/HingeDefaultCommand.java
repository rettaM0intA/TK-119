// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.enums.HingePosition;

public class HingeDefaultCommand extends CommandBase {

  boolean hold = false;
  int speedUpTick = 0;
  HingePosition previouPosition = HingePosition.Retracted;
  PIDController pid = new PIDController(0.0000087, 0.00000025, 0);
  /** Creates a new HingeDefaultCommand. */
  public HingeDefaultCommand() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hinge);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // if(RobotContainer.hingePos != previouPosition){
    //   hold = false;
    // }

    // if(RobotContainer.hinge.Pivot(RobotContainer.hingePos, hold)){
    //   hold = true;
    // }

    // previouPosition = RobotContainer.hingePos;

    if(RobotContainer.driver.getAButton()){
      speedUpTick = 0;
      if(RobotContainer.hinge.Pivot(HingePosition.Straight, false)){
        hold = true;
      }
      if(speedUpTick == 0){
        pid = new PIDController(0.0000087, 0.00000025, 0);
      }
      speedUpTick++;
    }else if(RobotContainer.driver.getBButton()){
      var d = pid.calculate(RobotContainer.hinge.hingeMotor.getSelectedSensorPosition(), 3000);

      RobotContainer.hinge.Pivot(d);

      hold = false;
    }else{
      RobotContainer.hinge.Pivot(0);
      hold = false;
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
