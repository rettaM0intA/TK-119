// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.ElevatorPosition;

public class ElevatorsSubsystem extends SubsystemBase {

  private int currentPID = -1;

  private WPI_TalonFX VerticalLeft = new WPI_TalonFX(12);
  private WPI_TalonFX VerticalRight = new WPI_TalonFX(13);

  //All the directional limits. up down out in.
  private final double verticalLimit = 70000;
  private final double highGoalPosition = 69000;
  private final double highSlowdownPosition = 67000;
  private final double shelfGoalPosition = 62000;
  private final double midGoalPosition = 58000;
  private final double groundLimit = 2000;

  /** Creates a new ElevatorsSubsystem. */
  public ElevatorsSubsystem() {

    VerticalLeft.setSelectedSensorPosition(0);
    VerticalRight.setSelectedSensorPosition(0);

    VerticalLeft.setInverted(false);
    VerticalRight.setInverted(false);

    VerticalLeft.setNeutralMode(NeutralMode.Brake);
    VerticalRight.setNeutralMode(NeutralMode.Brake);

    VerticalLeft.enableVoltageCompensation(false);
    VerticalRight.enableVoltageCompensation(false);

    VerticalRight.follow(VerticalLeft);

    PID(0);

    Move(0);
  }

  /**
   * Use to raise and lower the elevator with the input speed. Automatically stops before limits
   * @param speed Power to drive the motor with.
   */
  public void Move(double speed){
    if((speed > 0 && !TopLimitReached()) || (speed < 0 && !BottomLimitReached())){
      VerticalLeft.set(TalonFXControlMode.PercentOutput, speed);
    }else{
      VerticalLeft.set(TalonFXControlMode.PercentOutput, 0);
    }
    
  }

  public void voltageMove(double power){

    double voltage;

    if(Math.abs(power) > 4){
      voltage = Math.signum(power) * 4;
      VerticalLeft.setVoltage(voltage);
    }

    if(power > 0 && TopLimitReached()){
      VerticalLeft.setVoltage(1.1);
    }else{

      VerticalLeft.setVoltage(power);

    }


    // SmartDashboard.putNumber("LeftElevatorVoltage", power);
    // SmartDashboard.putNumber("RightElevatorVoltage", power);

  }


  private void PID(int PidType){

    if(PidType == 0 && currentPID != 0){
      VerticalLeft.config_kP(0, 0);
      VerticalLeft.config_kI(0, 0);
      VerticalLeft.config_kD(0, 0);
      VerticalLeft.config_kF(0, 0);
    }
  }

  public boolean TopLimitReached(){
    return VerticalLeft.getSelectedSensorPosition() >= verticalLimit;
  }

  public boolean HighReached(){
    return VerticalLeft.getSelectedSensorPosition() >= highGoalPosition;
  }

  public boolean HighChangePointReached(){
    return VerticalLeft.getSelectedSensorPosition() >= highSlowdownPosition;
  }

  public boolean MidReached(){
    return VerticalLeft.getSelectedSensorPosition() >= midGoalPosition;
  }

  public boolean ShelfReached(){
    return VerticalLeft.getSelectedSensorPosition() >= shelfGoalPosition;
  }

  public boolean BottomLimitReached(){
    return VerticalLeft.getSelectedSensorPosition() <= groundLimit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("LeftElevatorPosition", VerticalLeft.getSelectedSensorPosition());
    // SmartDashboard.putNumber("RightElevatorPosition", VerticalRight.getSelectedSensorPosition());
  }

public boolean goalReached() {
    switch (RobotContainer.elevatPos){
      case Floor:
        return BottomLimitReached();
      case Mid:
        return MidReached();
      case Shelf:
        return ShelfReached();
      case Top:
        return HighChangePointReached();
      default:
        return false;
    }
}
}
