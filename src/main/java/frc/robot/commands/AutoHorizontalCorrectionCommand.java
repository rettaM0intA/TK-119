// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Driveline;

public class AutoHorizontalCorrectionCommand extends CommandBase {

  Timer timer = new Timer();
  Timer timeLevel = new Timer();
  Timer isNotLevel = new Timer();
  double time;
  double speed = 0;
  double loweredSpeed = 0;
  static double rotation;
  static double startRotation;
  boolean stayStraight = false;
  boolean timerHasStarted = false;

  /** Creates a new HorizontalCorrectionCommand. */
  /**
   * This is the main balancing command.
   * 
   * @param _speed The factor multiplied by the pitch. Should be 0.012 when
   *               driving backwards and [get value] when driving forwards.
   * @param _time  The maximum duration of the command.
   */
  public AutoHorizontalCorrectionCommand(double _speed, double _time, double _loweredSpeed, boolean _stayStraight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    time = _time;
    speed = _speed;
    stayStraight = _stayStraight;
    loweredSpeed = _loweredSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startRotation = RobotContainer.driveline.getRobotAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (stayStraight) {

      rotation = (RobotContainer.driveline.getRobotAngle() - startRotation) * 0.02;
      if (Math.abs(rotation) > 0.1) {
        switch ((int) Math.signum((int) RobotContainer.driveline.getRobotAngle())) {
          case (1):
            rotation = 0.1;
            break;
          default:
            rotation = -0.1;
            break;
        }
      }
    } else {
      rotation = 0;
    }

    RobotContainer.driveline.drive(0, (RobotContainer.driveline.getRobotPitch() - 1.5) * speed, rotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveline.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // degrees forward or backward.
    boolean rtn = false;

    if (-2 < (RobotContainer.driveline.getRobotPitch() - 1.5)
        && (RobotContainer.driveline.getRobotPitch() - 1.5) < 2
        && timerHasStarted == false) {
      timeLevel.start();
      timerHasStarted = true;
    }

    if (timeLevel.get() > 3
        && (-2 < (RobotContainer.driveline.getRobotPitch() - 1.5)
            && (RobotContainer.driveline.getRobotPitch() - 1.5) < 2)
        || timer.hasElapsed(time)) {
          RobotContainer.driveline.drive(0, 0, 0, true);
      rtn = true;
    }

    boolean approachingForwards = false;

    if (RobotContainer.driveline.getRobotPitch() < -9 // if (RobotContainer.driveline.getRobotPitch() < 11.5 && RobotContainer.driveline.getRobotPitch() < -10
        || RobotContainer.driveline.getRobotPitch() > 10) {
      isNotLevel.start();
    } // -12, 13.5

    if (RobotContainer.driveline.getRobotPitch() - 1.5 > 0) {
      approachingForwards = true;
    } else if (RobotContainer.driveline.getRobotPitch() - 1.5 < 0) {
      approachingForwards = false;
    }

    if (approachingForwards) {
      if (isNotLevel.get() > 1.55) {
        speed = loweredSpeed; // 0.00675
      }
    } else {
      if (isNotLevel.get() > 0.9) {
        speed = loweredSpeed; // 0.00775
      }
    }
    
    SmartDashboard.putNumber("Time", isNotLevel.get());

    return rtn;
  }
}
