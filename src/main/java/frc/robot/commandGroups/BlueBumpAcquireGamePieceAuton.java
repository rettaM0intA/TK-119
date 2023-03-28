// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.commands.VisionAssistedGamepieceLocator;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueBumpAcquireGamePieceAuton extends SequentialCommandGroup {
  /** Creates a new BlueBumpAcquireGamePieceAuton. */

  /**
   * For blue alliance only.
   * <p>
   * Drops game piece, drives across the bump, picks up a cube, turns to face the
   * scoring area.
   */
  public BlueBumpAcquireGamePieceAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Scores cone on the top row
    addCommands(new ResetGyroCommand(),
        new ChangeIntakePositionsCommand(4),
        new IntakePositionsReachedCommand(3),
        new ClawHingeCommand(),
        new ResetSteerEncodersCommand(),
        new ParallelCommandGroup(new DrivelineDrivePIDCommand(0.015, 0, 0, 0, false, true),
            new ChangeIntakePositionsCommand(1)));

    // Drives backwards, leaving the Community, and turns to face a cube
    addCommands(new IntakePositionsReachedCommand(3),
        // TODO: determine safe kp for this command
        new DrivelineDrivePIDCommand(0.01, 156, 180, 5, true, true),
        new DrivelineTurnPIDCommand(-170, 3, true));

    // Adjusts with vision
    addCommands(new ResetGyroCommand(),
        new VisionAssistedGamepieceLocator(0),
        new ParallelCommandGroup(new ChangeIntakePositionsCommand(2)));

    // Drives forward and picks up the cube
    addCommands(new IntakePositionsReachedCommand(3),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.in),
        new ResetGyroCommand(),
        new DrivelineDrivePIDCommand(0.015, 48, 0, 3, true, true),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
        new ChangeIntakePositionsCommand(1),
        new IntakePositionsReachedCommand(3));

    // Turns to roughly face the scoring area
    addCommands(new DrivelineTurnPIDCommand(175, 3, false));

    // This cannot work. Needs to drive at an angle first.

    // // Moves towards scoring area
    // new DrivelineDrivePIDCommand(0, 0, 5, false));

    // // Uses vision to locate AprilTag and move towards it

    // // Scores game object (ball)
    // addCommands(new ChangeIntakePositionsCommand(3), // (wait)
    // new ChangeClawWheelDirectionCommand(ClawWheelDirection.out));
  }
}
