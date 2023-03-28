// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CrossChargeStationCommand extends CommandBase {
  Timer timer = new Timer();
  Timer timeLevel = new Timer();
  double time = 0;
  static double rotation;
  static double startRotation;
  boolean stayStraight = false;
  boolean hasReachedFloor = false;

  /** Creates a new CrossChargeStationCommand. */
  public CrossChargeStationCommand(double _time, boolean _stayStraight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    time = _time;
    stayStraight = _stayStraight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();

    startRotation = RobotContainer.driveline.getRobotAngle();

    RobotContainer.driveline.drive(0, -0.35, rotation, true);
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

    if (hasReachedFloor) {
      RobotContainer.driveline.drive(0, -0.1, rotation, true);
    } else if (RobotContainer.driveline.getRobotPitch() - 1.5 > 0) {
      RobotContainer.driveline.drive(0, -0.35, rotation, true); // -0.3
    } else if (RobotContainer.driveline.getRobotPitch() - 1.5 <= 0) {
      RobotContainer.driveline.drive(0, -0.4, rotation, true); // -0.35
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;

    if (RobotContainer.driveline.getRobotPitch() > 0.6
        && RobotContainer.driveline.getRobotPitch() < 2.6 && hasReachedFloor == false) {
      timeLevel.start();
      hasReachedFloor = true;
    }

    if (timeLevel.get() >= 1.5 || timer.get() >= time) {
      rtn = true;
    }

    return rtn;
  }
}
