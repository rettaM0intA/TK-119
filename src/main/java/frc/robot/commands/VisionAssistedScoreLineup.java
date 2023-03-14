// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.enums.CameraMode;

public class VisionAssistedScoreLineup extends CommandBase {

  int rightDistanceCounter = 0;

  /** Creates a new VisionAssistedItemPlacement. */
  public VisionAssistedScoreLineup() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    addRequirements(RobotContainer.camera);

    RobotContainer.camera.CameraMode(CameraMode.aprilTag);

    rightDistanceCounter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightDistanceCounter = 0;

    RobotContainer.camera.CameraMode(CameraMode.aprilTag);
    
    RobotContainer.driveline.straightenSteerMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double horizontal = 0;
    double rotate = 0;
    
    horizontal = (RobotContainer.camera.tx.getDouble(0) + 4) * .02;

    if(horizontal > .05)
      horizontal = .05;
    else if(horizontal < -.05)
      horizontal = -.05;

    if(Math.abs(RobotContainer.driveline.getRobotAngle()) > 2)
      rotate = RobotContainer.driveline.getRobotAngle() * .02;

    if(rotate > .05)
      rotate = .05;
    else if(rotate < -.05)
      rotate = -.05;

    if(RobotContainer.camera.TargetLocated() && 
    RobotContainer.camera.tx.getDouble(-500) > -4 && 
    RobotContainer.camera.tx.getDouble(500) < -2 && 
    Math.abs(RobotContainer.driveline.getRobotAngle()) < 2){

        RobotContainer.driveline.drive(0, 0, 0, false);
        rightDistanceCounter += 1;

    }else if(RobotContainer.camera.TargetLocated()){

      RobotContainer.driveline.drive(horizontal, 0, rotate, false);

      // if(RobotContainer.camera.tx.getDouble(-500) > 3){
      //   RobotContainer.driveline.drive(0.15, 0, 0, false);
      // }else if(RobotContainer.camera.tx.getDouble(500) < -3){
      //   RobotContainer.driveline.drive(-.15, 0, -.0, false);
      // }
    }else{
      RobotContainer.driveline.drive(0, 0, rotate, false);
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
