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
public class DrivelineRotationalDrivePIDCommand extends PIDCommand {
  /** Creates a new AutoDrivePID. */

  Timer timer;
  double inches;
  double time;

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
   */
  public DrivelineRotationalDrivePIDCommand(double _inches, double _angle, double _rotation, double _rotAngle, double _time, boolean _fieldOriented) {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0.0025, 0.0025), // 0.015 for balancing
        // This should return the measurement
        () -> RobotContainer.driveline.getAverageDistanceInInches(),
        // This should return the setpoint (can also be a constant)
        () -> _inches,
        // This uses the outputs
        output -> {
          // Use the output here

          // if (_angle >= 0) {
            RobotContainer.driveline.drive(Math.sin(_angle * Math.PI / 180) * output,
            Math.cos(_angle * Math.PI / 180) * output, _rotation, _fieldOriented);
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

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.RobotContainer;

// // NOTE: Consider using this command inline, rather than writing a subclass.
// For more
// // information, see:
// //
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class DrivelineDrivePIDCommand extends PIDCommand {
// /** Creates a new DriveLineDrivePIDCommand. */

// double m_timeout;
// Timer timer;
// boolean isFinished = false;

// @Override
// public void initialize() {
// super.initialize();
// timer = new Timer();
// timer.start();
// RobotContainer.m_driveline.resetMotors();
// }

// public DrivelineDrivePIDCommand(double _inches, double _time) {
// super(
// // The controller that the command will use
// new PIDController(0.025, 0.000125, 0.001525),
// // This should return the measurement (current position)
// () -> RobotContainer.m_driveline.getInches(),
// // This should return the setpoint (goal position)
// () -> _inches,
// // This uses the output
// output -> {
// RobotContainer.m_driveline.driveAuto(output, 0, 1);
// // Use the output here
// });
// // Use addRequirements() here to declare subsystem dependencies.
// addRequirements(RobotContainer.m_driveline);
// m_timeout = _time;
// // Configure additional PID options by calling `getController` here.
// getController().setSetpoint(_inches);

// getController().setTolerance(5);

// getController().setIntegratorRange(0, 1);

// SmartDashboard.putNumber("P", getController().getP());
// SmartDashboard.putNumber("I", getController().getI());
// SmartDashboard.putNumber("D", getController().getD());
// SmartDashboard.putNumber("Velocity Error",
// getController().getVelocityError());
// }

// // Returns true when the command should end.
// @Override
// public boolean isFinished() {
// if (this.m_controller.atSetpoint() || timer.hasElapsed(m_timeout)) {
// SmartDashboard.putString("has_finished", "is_done");
// RobotContainer.m_driveline.driveAuto(0, 0, 0);
// return true;
// } else
// return false;
// }
// }

// // file > preferences > settings > text editor > font
// // Research gain scheduling as a means to overcome issues with overshooting
// when goal distance is changed.