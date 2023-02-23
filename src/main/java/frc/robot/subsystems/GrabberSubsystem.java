// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class GrabberSubsystem extends SubsystemBase {

//   //Define the motors used
//   private TalonFX rollorMotorLeft = new TalonFX(14);
//   private TalonFX rollorMotorRight = new TalonFX(15);

//   private DoubleSolenoid pincher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);


//   /** Creates a new GrabberSubsystem. */
//   public GrabberSubsystem() {

//   }

//   /**
//    * Will spin at a set velocity in the input direction.
//    * @param spinIn set true to have them spin objects in. Have false to spin objects out.
//    */
//   public void Spin(boolean spinIn){
//     if(spinIn){
//       rollorMotorLeft.set(TalonFXControlMode.Velocity, 5);
//       rollorMotorRight.set(TalonFXControlMode.Velocity, 5);
//     }else{
//       rollorMotorLeft.set(TalonFXControlMode.Velocity, -5);
//       rollorMotorRight.set(TalonFXControlMode.Velocity, -5);
//     }
//   }

//   /**
//    * Will spin at the specified power output.
//    * @param speed Speed of the motors as a percentage of max speed
//    */
//   public void Spin(double speed){
//     rollorMotorLeft.set(TalonFXControlMode.Current, speed);
//     rollorMotorRight.set(TalonFXControlMode.Current, speed);
//   }

//   /**
//    * Used to open and close the intake with the pnumatic piston
//    * @param isClosed set true to make the claw close. True to push it open.
//    */
//   public void Piston(boolean isClosed){
//     if(isClosed){
//       pincher.set(Value.kReverse);
//     }else{
//       pincher.set(Value.kForward);
//     }
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
