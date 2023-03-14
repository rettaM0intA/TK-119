// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CrossChargeStationCommand extends CommandBase {
  Timer timer = new Timer();
  Timer hasReachedFloor = new Timer();
  double time = 0;
  boolean hasReachedFloorBegun = false;

  /** Creates a new CrossChargeStationCommand. */
  public CrossChargeStationCommand(double _time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveline);
    time = _time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.driveline.getRobotPitch() - 1.5 > 0) {
      RobotContainer.driveline.drive(0, -0.3, 0, true);
    } else if (RobotContainer.driveline.getRobotPitch() - 1.5 <= 0) {
      RobotContainer.driveline.drive(0, -0.25, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;

    if (timer.get() >= time) {
      rtn = true;
    }

    if (timer.get() >= 3 && RobotContainer.driveline.getRobotPitch() > 0.6 && RobotContainer.driveline.getRobotPitch() < 2.6 && hasReachedFloorBegun == false) {
      hasReachedFloor.start();
      hasReachedFloorBegun = true;
    }

    if (hasReachedFloor.get() >= 0.5) {
      rtn = true;
    }

    return rtn;
  }
}
