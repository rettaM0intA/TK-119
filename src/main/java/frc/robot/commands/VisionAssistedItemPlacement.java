// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class VisionAssistedItemPlacement extends CommandBase {

  int rightDistanceCounter = 0;

  /** Creates a new VisionAssistedItemPlacement. */
  public VisionAssistedItemPlacement() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    addRequirements(RobotContainer.camera);

    rightDistanceCounter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightDistanceCounter = 0;

    RobotContainer.camera.CameraMode(true);
    
    RobotContainer.driveline.straightenSteerMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.camera.TargetLocated() && RobotContainer.camera.tx.getDouble(500) > -3 && RobotContainer.camera.tx.getDouble(500) < 3){
      RobotContainer.driveline.drive(0, .2, 0, false);
      if(RobotContainer.camera.ta.getDouble(-500) > 2.2){
        RobotContainer.driveline.drive(0, 0, 0, false);
        rightDistanceCounter += 4;
      }

    }else if(RobotContainer.camera.TargetLocated()){
      if(RobotContainer.camera.tx.getDouble(0) > 3){
        RobotContainer.driveline.drive(0, 0, .2, false);
      }else if(RobotContainer.camera.tx.getDouble(0.0) < -3){
        RobotContainer.driveline.drive(0, 0, -.2, false);
      }
    }else{
      RobotContainer.driveline.drive(0, 0, .1, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(RobotContainer.camera.TargetLocated() && RobotContainer.camera.tx.getDouble(0) > -1 && RobotContainer.camera.tx.getDouble(0) < 1){
    //   RobotContainer.driveline.drive(0, 0, 0, false);
    //   return true;
    // }

    if(rightDistanceCounter >= 4){
      return true;
    }
    return false;
  }
}
