// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoHorizontalCorrectionCommand;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.CrossChargeStationCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakeFastModeCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.enums.ClawWheelDirection;

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

    // Scores cone on high row
    // TODO: save time on cone placement
    addCommands(new ResetGyroCommand(),
        new IntakeFastModeCommand(true),
        new ChangeIntakePositionsCommand(4),
        new IntakePositionsReachedCommand(3),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
        new ClawHingeCommand(),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),        new ResetSteerEncodersCommand(),
        new ParallelCommandGroup(new DrivelineDrivePIDCommand(0.0175, 0, 0, 0, true, true),
            new ChangeIntakePositionsCommand(1)));

    addCommands(new IntakePositionsReachedCommand(3));

    // Drives across the Charge Station
    // TODO: back up slightly farther
    addCommands(new DrivelineDrivePIDCommand(0.018, 48, 180, 3, true, true),
        // Time may still need to be slightly less on stopping at the end
        // TODO: manually time how long it takes to get halfway across and all the way
        new CrossChargeStationCommand(7, true));

    // addCommands(new DrivelineTurnPIDCommand(0, 3, true));

    // addCommands(new DrivelineDrivePIDCommandCodyVer(0.005, 48, 180, 5, true,
    // true));

    // This works for approaching the Charge Station with the front of the robot:
    // TODO: get balance immediately
    addCommands(new DrivelineDrivePIDCommand(0.018, 36, 0, 3, true, true),
        new AutoHorizontalCorrectionCommand(0.01415, 5, 0, true));
  }

  // Straighten, drive forward w/PID and use AutoHorizontalCorrectionCommand
}
