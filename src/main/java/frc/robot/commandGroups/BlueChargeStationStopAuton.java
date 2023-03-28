// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoHorizontalCorrectionCommand;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.HorizontalCorrectionPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueChargeStationStopAuton extends SequentialCommandGroup {
  /** Creates a new CompetitionAuton_4. */

  /**
   * Drops game piece, drives onto the Charge Station, stays there.
   */
  public BlueChargeStationStopAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Scores cone on the top row
    addCommands(new ResetGyroCommand(),
        new ChangeIntakePositionsCommand(4),
        new IntakePositionsReachedCommand(3),
        // new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
        new ClawHingeCommand(),
        // new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
        new ResetSteerEncodersCommand(),
        new ParallelCommandGroup(new DrivelineDrivePIDCommand(0.015, 0.0001, 0, 1, true, false),
            new ChangeIntakePositionsCommand(1)));

    // Drives backward onto the Charge Station and balances
    addCommands(new IntakePositionsReachedCommand(3),
        new DrivelineDrivePIDCommand(0.01, 60, 180, 5, true, true),
        new AutoHorizontalCorrectionCommand(0.016, 5, 0.00685, true)); // 0.01435
  } // 0.016
}
