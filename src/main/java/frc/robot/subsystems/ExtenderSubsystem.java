// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode; //DO NOT COMMENT OR DELETE THIS LINE. Cody will not be happy :(
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ExtenderPosition;

public class ExtenderSubsystem extends SubsystemBase {

  private TalonFX leftExtender = new TalonFX(10);
  private TalonFX rightExtender = new TalonFX(11);

  //Real out limit = 32000
  private double outChangePoint = 27500;
  private double outSecondChangePoint = 30000;
  private double outLimit = 32000;
  private double inChangePoint = 4000;
  private double inSecondChangePoint = 500;
  private double inLimit = 100;

  private boolean goingOut = false;
  private boolean goingIn = false;

  private int powerFactor = 0;
  private int initialPowerFacter = 45;

  private double inDivisor = 90;
  private double outDivisor = 90;

  /** Creates a new ElavatorHorizontalSubsystem. */
  public ExtenderSubsystem() {

    // leftExtender.setSelectedSensorPosition(0);
    // rightExtender.setSelectedSensorPosition(0);

    // leftExtender.setNeutralMode(NeutralMode.Brake);
    // rightExtender.setNeutralMode(NeutralMode.Brake);

    Hold();

    Pid(0);

  }

  /**
   * 
   * @param outOrIn make true to send out the intake. Set false to pull it back in.
   */
  public void Extend(boolean outOrIn){
    
    if(outOrIn){
      goingOut = false;
      if(!goingIn){
        powerFactor = 0;
        goingIn = true;
        goingOut = false;
      }
      if(getPositions() <= outChangePoint){

        //Use an increasing factor to prevent a rough start
        if(powerFactor <= inDivisor){
          powerFactor++;
          leftExtender.set(TalonFXControlMode.PercentOutput, 0.3 * powerFactor / outDivisor);
          rightExtender.set(TalonFXControlMode.PercentOutput, 0.3 * powerFactor / outDivisor);
        }else{
          //drive as though the fraction = 1 to prevent going too fast
          leftExtender.set(TalonFXControlMode.PercentOutput, 0.3);
          rightExtender.set(TalonFXControlMode.PercentOutput, 0.3);
        }

        // leftExtender.set(TalonFXControlMode.Position, outLimit);
      }else if(getPositions() < outSecondChangePoint){
        if(powerFactor > outDivisor / 2){

          powerFactor -= 6;
          leftExtender.set(TalonFXControlMode.PercentOutput, 0.21 * powerFactor / outDivisor);
          rightExtender.set(TalonFXControlMode.PercentOutput, 0.21 * powerFactor / outDivisor);
        }else{

          leftExtender.set(TalonFXControlMode.PercentOutput, 0.1);
          rightExtender.set(TalonFXControlMode.PercentOutput, 0.1);
        }
      }else{
        leftExtender.set(TalonFXControlMode.Position, outLimit);
        rightExtender.set(TalonFXControlMode.Position, outLimit);
      }
      return;
    }
    if(!outOrIn){
      goingIn = false;
      if(!goingOut){
        powerFactor = initialPowerFacter;
        goingOut = true;
      }
      if(getPositions() >= inChangePoint){
        
        //Use an increasing factor to prevent a rough start
        if(powerFactor <= inDivisor){
          powerFactor++;
          leftExtender.set(TalonFXControlMode.PercentOutput, -0.3 * powerFactor / inDivisor);
          rightExtender.set(TalonFXControlMode.PercentOutput, -0.3 * powerFactor / inDivisor);
        }else{
          //drive as though the fraction = 1 to prevent going too fast
          leftExtender.set(TalonFXControlMode.PercentOutput, -0.3);
          rightExtender.set(TalonFXControlMode.PercentOutput, -0.3);
        }

      }else if(getPositions() > inSecondChangePoint){
        //keep the extender in the robot
        leftExtender.set(TalonFXControlMode.PercentOutput, -.13);
        rightExtender.set(TalonFXControlMode.PercentOutput, -.13);
      }else if(getPositions() > inLimit){
        //gently reach the limit
        leftExtender.set(TalonFXControlMode.PercentOutput, -.08);
        rightExtender.set(TalonFXControlMode.PercentOutput, -.08);

      }else{
        //Make the motors idle while at the right inner position
        leftExtender.set(TalonFXControlMode.PercentOutput, -.06);
        rightExtender.set(TalonFXControlMode.PercentOutput, -.06);
      }

      return;
    }

    Hold();
  }

  public void Hold(){
    leftExtender.set(TalonFXControlMode.PercentOutput, 0);
    rightExtender.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getPositions(){
    return ((leftExtender.getSelectedSensorPosition() + rightExtender.getSelectedSensorPosition()) / 2);
  }

  private void Pid(int pidType){
    if(pidType == 0){
      leftExtender.config_kP(0, 0.057);
      leftExtender.config_kI(0, 0);
      leftExtender.config_kD(0, 0);
      leftExtender.config_kF(0, 0.0012);
      
      rightExtender.config_kP(0, 0.057);
      rightExtender.config_kI(0, 0);
      rightExtender.config_kD(0, 0);
      rightExtender.config_kF(0, 0.0012);
    }
  }

  public boolean AtDestination(){
    if(RobotContainer.extendPos == ExtenderPosition.Extended && getPositions() > outChangePoint){
      return true;
    }

    if(RobotContainer.extendPos == ExtenderPosition.Retracted && getPositions() < inSecondChangePoint){
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("leftExtenderPos", leftExtender.getSelectedSensorPosition());
    // SmartDashboard.putNumber("rightExtenderPos", rightExtender.getSelectedSensorPosition());

    
    // SmartDashboard.putNumber("leftExtender Output", leftExtender.getMotorOutputPercent());
    // SmartDashboard.putNumber("rightExtender Output", rightExtender.getMotorOutputPercent());
  }

  public boolean goalReached() {
      switch (RobotContainer.extendPos){
        case Extended:
          return getPositions() >= outLimit - 1000;
        case Retracted:
          return getPositions() <= inLimit + 1000;
        default:
          return false;
      }
  }
}
