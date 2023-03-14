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
public class HorizontalCorrectionPIDCommand extends PIDCommand {
  /** Creates a new HorizontalCorrectionPIDCommand. */

  Timer timer;
  double timeout;

  @Override
  public void initialize() {
    super.initialize();
    Timer timer = new Timer();
    timer.start();
  }

  /**
   * This command allows the robot to balance on the Charge Station.
   * @param _time maximum duration of the command
   */
  public HorizontalCorrectionPIDCommand(double _time) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, 0, 0),
        // This should return the measurement
        () -> RobotContainer.driveline.getRobotPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          RobotContainer.driveline.drive(0, -output, 0, false);
        });
        addRequirements(RobotContainer.driveline);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    timeout = _time;
    this.m_controller.setTolerance(0.5);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;
    boolean robotIsLevel = false;
    if (this.m_controller.atSetpoint() || timer.hasElapsed(timeout)) {
      rtn = true;
      robotIsLevel = true;
    } else {
      robotIsLevel = false;
    }
    SmartDashboard.putBoolean("Robot is Level", robotIsLevel);
    return rtn;
  }
}
