// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorsSubsystem extends SubsystemBase {

  private TalonFX VerticalLeft = new TalonFX(12);
  private TalonFX VerticalRight = new TalonFX(13);

  //All the directional limits. up down out in.
  private double verticalLimit = 100;
  private double groundLimit = 0;

  /** Creates a new ElevatorsSubsystem. */
  public ElevatorsSubsystem() {}

  /**
   * Use to raise and lower the elevator by a predetermined rate. Automatically stops before limits
   * @param direction Set true to go up. False to go down
   */
  public void RaiseLower(boolean direction){

    if(direction && !TopLimitReached()){
      VerticalLeft.set(TalonFXControlMode.Velocity, 5);
      VerticalRight.set(TalonFXControlMode.Velocity, 5);
    }else if(!direction && !BottomLimitReached()){
      VerticalLeft.set(TalonFXControlMode.Velocity, -5);
      VerticalRight.set(TalonFXControlMode.Velocity, -5);
    }

  }

  /**
   * Use to raise and lower the elevator with the input speed. Automatically stops before limits
   * @param speed Power to drive the motor with.
   */
  public void RaiseLower(double speed){
    if((speed > 0 && !TopLimitReached()) || (speed < 0 && !BottomLimitReached())){
      VerticalLeft.set(TalonFXControlMode.Velocity, speed);
      VerticalRight.set(TalonFXControlMode.Velocity, speed);
    }
    
  }

  public boolean TopLimitReached(){
    return VerticalLeft.getSelectedSensorPosition() >= verticalLimit;
  }

  public boolean BottomLimitReached(){
    return VerticalLeft.getSelectedSensorPosition() <= groundLimit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
