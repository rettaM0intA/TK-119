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
  int speedUpTick = 20;

  HingePosition previousPosition = HingePosition.Retracted;
  
  PIDController pid = new PIDController(0.0000125, 0.00000025, 0);

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

    if((previousPosition == HingePosition.Floor && RobotContainer.hingePos == HingePosition.Straight) ||
    (previousPosition != RobotContainer.hingePos && RobotContainer.hingePos == HingePosition.Retracted)){
      speedUpTick = 0;
    }
    previousPosition = RobotContainer.hingePos;

    if(RobotContainer.hingePos == HingePosition.Retracted){
      if(speedUpTick < 2){

        RobotContainer.hinge.Pivot(-.4);
        speedUpTick++;

      }else{
        var d = pid.calculate(RobotContainer.hinge.hingeMotor.getSelectedSensorPosition(), 2000);
        
        RobotContainer.hinge.Pivot(d);
        
      }
      

      hold = false;
    }else{

      // if(speedUpTick < 10 && RobotContainer.hingePos == HingePosition.Straight){
      //   RobotContainer.hinge.Pivot(-.4);
      //   speedUpTick++;
      // }else
       if(RobotContainer.hinge.Pivot(RobotContainer.hingePos, hold)){
        hold = true;
      }

      //Reset the pid for retracting. Yes, this does something
      pid = new PIDController(0.0000125, 0.00000025, 0);
    }

      // RobotContainer.hinge.Pivot(RobotContainer.driver.getRightTriggerAxis() - RobotContainer.driver.getLeftTriggerAxis());

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
