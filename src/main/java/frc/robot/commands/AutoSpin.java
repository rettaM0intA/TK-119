// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoSpin extends CommandBase {

  boolean isFinished = false;
  boolean isRight = false;
  double speed = 0; // Corresponds with input.
  double startDegree = 0;
  double goalDegree = 0; // Corresponds with input.
  double currentDegree = 0;
  int buffer = 0;

  /** Creates a new AutoSpin. */
  public AutoSpin(double m_goalDegree, double m_speed, boolean m_isRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    
    speed = m_speed * 0.01;
    isRight = m_isRight;

    
    startDegree = RobotContainer.driveline.getGyro().getAngle();

    if(isRight){
      goalDegree = m_goalDegree /*+ startDegree*/;
    }else{
      goalDegree = -m_goalDegree /*+ startDegree*/;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalDegree = goalDegree + RobotContainer.driveline.getGyro().getAngle();
    buffer = 0;

    RobotContainer.driveline.drive(0, 0, 0, false);

    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Buffer", buffer);

    buffer += 1;

    currentDegree = RobotContainer.driveline.getGyro().getAngle();

    // if(goalDegree > 0){
    if(isRight){
      if(buffer > 20){
        RobotContainer.driveline.drive(0, 0, speed, false);
      }else{
        RobotContainer.driveline.drive(0, 0, 0.03, false);
      }
    
      if(currentDegree > goalDegree -35 && speed > 0.2){
        speed *= 0.5;
      }

      if(currentDegree >= goalDegree){
        isFinished = true;
      }

    }else{
        if(buffer > 20){
          RobotContainer.driveline.drive(0, 0, -speed, false);
        }else{
          RobotContainer.driveline.drive(0, 0, -0.03, false);
        }
      
        if(currentDegree < goalDegree + 35 && speed > 0.2){
          speed *= 0.5;
        }
  
        if(currentDegree <= goalDegree){
          isFinished = true;
        }
  
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(isFinished){
        // RobotContainer.m_chassisSubsystem.resetGyro();
        RobotContainer.driveline.resetSwerveDriveEncoders();
        RobotContainer.driveline.drive(0, 0, 0, false);
        isFinished = false;
        return true;
      }
      return false;
  }
}
