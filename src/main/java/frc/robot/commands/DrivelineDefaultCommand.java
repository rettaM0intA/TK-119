// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ActiveMode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.OI;
import frc.robot.subsystems.Driveline;

public class DrivelineDefaultCommand extends CommandBase {

  boolean straightenMotors = true;

  /** Creates a new DrivelineDefaultCommand. */
  public DrivelineDefaultCommand(Driveline _driveline) {
    addRequirements(_driveline);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(straightenMotors){
      RobotContainer.driveline.straightenSteerMotors();
      straightenMotors = false;
    }
    
    // // if(RobotContainer.activeMode == ActiveMode.autonomous){
    // Read Joystick values for X, Y and rotation
    double x = RobotContainer.driver.getLeftX();
    double y = -RobotContainer.driver.getLeftY();
    double twist = RobotContainer.driver.getRightX() * 0.7;

    // Limit the X,Y Rotation values so minor changes do not make motors move
    x =  Util.deadband(x, OI.kDeadband);//(Math.abs(x) > OI.kDeadband) ? x - OI.kDeadband : 0;
    y =  Util.deadband(y, OI.kDeadband);//(Math.abs(y) > OI.kDeadband) ? y - OI.kDeadband : 0;
    twist = Util.deadband(twist, OI.kDeadband);// (Math.abs(twist) > OI.kDeadband) ? twist - OI.kDeadband : 0;

    if(RobotContainer.driveline.getFieldOrientedModeActive()){
      RobotContainer.driveline.drive(x, y, twist, 180);
    }else{
      RobotContainer.driveline.drive(x, y, twist, false);
    }

    

    // if(RobotContainer.driver.getYButtonPressed()){
    //   RobotContainer.driveline.straightenSteerMotors();
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
