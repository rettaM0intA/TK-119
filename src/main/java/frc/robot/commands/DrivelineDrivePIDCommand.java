// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenixpro.configs.OpenLoopRampsConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivelineDrivePIDCommand extends PIDCommand {
  /** Creates a new AutoDrivePID. */

  Timer timer;
  double inches;
  double time;
  static double rotation;

  @Override
  public void initialize() {
    super.initialize();
    timer = new Timer();
    timer.start();
    RobotContainer.driveline.resetSwerveDriveEncoders();
  }

  /**
   * This is the main driving command used by the robot during the autonomous period.
   * @param _inches distance in inches
   * @param _angle angle to move at relative to the front of the robot (from -180 to 180)
   * @param _time maximum duration of the command
   * @param _fieldOriented whether field-oriented mode is enabled
   * @param _stayStraight wether the robot should try to stay at the starting rotation or not.
   */
  public DrivelineDrivePIDCommand(double _kp, double _inches, double _angle, double _time, boolean _fieldOriented, boolean _stayStraight) {
    super(
        // The controller that the command will use
        new PIDController(_kp, 0.0025, 0.0025), // kp: 0.015 for balancing (front and back end first), 0.004 for backup on bump side auton
        // This should return the measurement
        () -> RobotContainer.driveline.getAverageDistanceInInches(),
        // This should return the setpoint (can also be a constant)
        () -> _inches,
        // This uses the outputs
        output -> {
          // Use the output here

          if(_stayStraight){
            
            rotation = RobotContainer.driveline.getRobotAngle() * .02;
            if(Math.abs(rotation) > 0.1){
              switch((int)Math.signum((int)RobotContainer.driveline.getRobotAngle())){
                case (1):
                rotation = .1;
                break;
                default:
                rotation = -.1;
                break;
              }
            }
            
            RobotContainer.driveline.drive(Math.sin(_angle * Math.PI / 180) * output,
            Math.cos(_angle * Math.PI / 180) * output, rotation, _fieldOriented);
          }else{
            RobotContainer.driveline.drive(Math.sin(_angle * Math.PI / 180) * output,
            Math.cos(_angle * Math.PI / 180) * output, 0, _fieldOriented);

          }

          // if (_angle >= 0) {
          // } else if (_angle < 0) {
          //   RobotContainer.driveline.drive(Math.sin(_angle * Math.PI / 180) * output,
          //   Math.cos(_angle * Math.PI / 180) * output, 0, _fieldOriented); // goes down and to the right
          // }

        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(RobotContainer.driveline);
    inches = _inches;
    time = _time;
    this.m_controller.setIntegratorRange(-0.5, 0.5);
    this.m_controller.setTolerance(1, 10 / 0.02);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;
    if (inches == 0) {
      RobotContainer.driveline.drive(0, 0, 0, false);
      rtn = true;
    }
    if (this.m_controller.atSetpoint() || timer.hasElapsed(time)) {
      rtn = true;
    }
    return rtn;
  }
}
