// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.commands.OrientWheelsCommand;
import frc.robot.commands.VisionAssistedGamepieceLocator;
import frc.robot.commands.WaitCommand;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueNoBumpAcquireGamePieceAuton extends SequentialCommandGroup {
    /** Creates a new CompetitionAuton_2. */

    /**
     * For blue alliance only.
     * <p>
     * Drops game piece, drives out on the side of the Charge Station without the
     * bump, obtains game piece, turns back towards scoring area.
     */
    public BlueNoBumpAcquireGamePieceAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        // Scores cone on the top row
        addCommands(new ChangeIntakePositionsCommand(4),
                new ResetGyroCommand(),
                new IntakePositionsReachedCommand(3),
                new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
                new ClawHingeCommand(),
                new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),                
                new ResetSteerEncodersCommand(),
                new ParallelCommandGroup(new DrivelineDrivePIDCommand(0.015, 0, 180, 1, true, true),
                        new ChangeIntakePositionsCommand(1)));

        // Drives backwards, leaving the Community, and turns to face a cube
        addCommands(new IntakePositionsReachedCommand(3),
                new DrivelineDrivePIDCommand(0.015, 132, 180, 5, true, true),
                new DrivelineTurnPIDCommand(170, 3, true));

        // Adjusts with vision
        addCommands(new VisionAssistedGamepieceLocator(170),
                // new ChangeClawWheelDirectionCommand(ClawWheelDirection.in));
                new ChangeIntakePositionsCommand(2),
                new DrivelineDrivePIDCommand(0.0125, 0.5, 0, 0.5, false, true));

        // Drives forward and picks up the cube
        addCommands(new IntakePositionsReachedCommand(3),
                new ChangeClawWheelDirectionCommand(ClawWheelDirection.in),
                new DrivelineDrivePIDCommand(0.0125, 48,
                        0, 3, false, true),
                new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
                new ChangeIntakePositionsCommand(1),
                new IntakePositionsReachedCommand(3));

        // Turns to roughly face the scoring area
        addCommands(new DrivelineTurnPIDCommand(0, 3, true));

        // // Moves towards scoring area
        // new DrivelineDrivePIDCommand(0, 0, 5, false));

        // // Uses vision to locate AprilTag and move towards it

        // // Scores game object (ball)
        // addCommands(new ChangeIntakePositionsCommand(3), // (wait)
        // new ChangeClawWheelDirectionCommand(ClawWheelDirection.out));

    }
}

// Notes: Figure out field-oriented mode issue with Limelight turning