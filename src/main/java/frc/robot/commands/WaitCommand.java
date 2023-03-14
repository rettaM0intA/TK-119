// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {

  Timer timer;
  double time;

  /** Creates a new WaitCommand. */
  public WaitCommand(double _time) {
    // Use addRequirements() here to declare subsystem dependencies.

    time = _time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rtn = false;
    if (timer.hasElapsed(time)) {
      rtn = true;
    }
    // SmartDashboard.putBoolean("Wait Command Has Ended", rtn);
    return rtn;
  }
}
