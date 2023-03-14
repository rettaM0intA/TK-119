// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

  //Compressor for pnumatics
  public Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  
  DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  
  private TalonFX leftRoller = new TalonFX(14);
  private TalonFX rightRoller = new TalonFX(15);

  boolean compressorEnabled = true;
  public int clawMovements = 0;
  public boolean activated = false;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {

    // compressor.enableDigital();
    compressor.disable();

    clawSolenoid.set(Value.kReverse);

    leftRoller.setInverted(false);
    rightRoller.setInverted(true);

    leftRoller.setNeutralMode(NeutralMode.Brake);
    rightRoller.setNeutralMode(NeutralMode.Brake);

    Hold();
  }

  public void Spin(double speed){
    leftRoller.set(TalonFXControlMode.PercentOutput, speed);
    rightRoller.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void Hold(){
    leftRoller.set(TalonFXControlMode.PercentOutput, 0);
    rightRoller.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void Hinge(boolean isClosed){
    if(isClosed){
      clawSolenoid.set(Value.kReverse);
    }else{
      clawSolenoid.set(Value.kForward);
    }

  }

  public void switchCompressorMode() {
    if(compressorEnabled){
      compressor.disable();
    }else{
      compressor.enableDigital();
    }

    compressorEnabled = !compressorEnabled;

  }

  public void incrementClawMovements(){
    clawMovements++;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putBoolean("Compressor Active", compressor.isEnabled());
    SmartDashboard.putBoolean("Compressor is Enabled", compressorEnabled);
    SmartDashboard.putNumber("Times claw has moved", clawMovements);
  }

}
