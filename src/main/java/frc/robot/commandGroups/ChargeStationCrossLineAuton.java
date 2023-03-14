// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoHorizontalCorrectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.CrossChargeStationCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationCrossLineAuton extends SequentialCommandGroup {
  /** Creates a new CompetitionAuton_4. */

  /**
   * Drops game piece, crosses the Charge Station (leaving the community), drives
   * back onto the Charge Station and balances.
   */
  public ChargeStationCrossLineAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // // Scores cone on top row
    // addCommands(new ResetGyroCommand(),
    // new ChangeIntakePositionsCommand(4),
    // new IntakePositionsReachedCommand(3),
    // new ClawHingeCommand(),
    // new ResetSteerEncodersCommand(),
    // new ParallelCommandGroup(new DrivelineDrivePIDCommand(0, 0, 0, false), new
    // ChangeIntakePositionsCommand(1)));

    // Drives across the Charge Station
    addCommands(new DrivelineDrivePIDCommand(0.015, 48, 180, 3, true),
        new CrossChargeStationCommand(7));

        // TODO: implement straightening after crossing Charge Station
        // addCommands(new DrivelineTurnPIDCommand(-RobotContainer.driveline.getRobotAngle(), 3, true));

    // // This works for going forward:
    // addCommands(new ResetGyroCommand(),
    // new DrivelineDrivePIDCommand(36, 0, 3, true),
    // new AutoHorizontalCorrectionCommand(0.014, 5));
  }

  // Straighten, drive forward w/PID and use AutoHorizontalCorrectionCommand
}
