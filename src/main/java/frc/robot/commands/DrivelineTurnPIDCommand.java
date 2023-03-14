// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Driveline;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivelineTurnPIDCommand extends PIDCommand {
  /** Creates a new DrivelineTurnCommand. */

  Timer timer;
  double goalAngle;
  double time;

  @Override
  public void initialize() {
    super.initialize();
    timer = new Timer();
    timer.start();
    RobotContainer.driveline.resetGyro();
    RobotContainer.driveline.resetSwerveDriveEncoders();
    RobotContainer.driveline.straightenSteerMotors();
    // SmartDashboard.putBoolean("Continuous Input Enabled", this.m_controller.isContinuousInputEnabled());
  }

  /**
   * This is the main turning command used by the robot during the autonomous period.
   * @param _goalAngle angle to turn to in degrees
   * @param _time maximum duration of the command
   * @param _fieldOriented whether field-oriented mode is enabled
   */
  public DrivelineTurnPIDCommand(double _goalAngle, double _time, boolean _fieldOriented) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0.025, 0.0025), // period = 0.02
        // This should return the measurement
        () -> RobotContainer.driveline.getRobotAngle(),
        // This should return the setpoint (can also be a constant)
        () -> _goalAngle,
        // This uses the output
        output -> { // FR spins around completely. How to optimize?
          // if (output >= 0.25) {
          //   RobotContainer.driveline.drive(0, 0, 0.25, _fieldOriented);
          // } else {
            RobotContainer.driveline.drive(0, 0, -output, _fieldOriented);
          // }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(RobotContainer.driveline);
    goalAngle = _goalAngle;
    time = _time;
    this.m_controller.setIntegratorRange(-0.5, 0.5);
    this.m_controller.setTolerance(1, 10 / 0.02); // 90 degrees / 2 s * 0.02 s
    this.m_controller.disableContinuousInput(); // position error = current distance away from setpoint
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;
    if (this.m_controller.atSetpoint() || timer.hasElapsed(time)) {
      rtn = true;
    }
    // SmartDashboard.putBoolean("DrivelineTurnPIDCommand has finished", rtn);
    return rtn;
  }
}
