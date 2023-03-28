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
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpStopAuton extends SequentialCommandGroup {
    /** Creates a new CompetitionAuton_1. */

    /**
     * Drops game piece, drives out of the Community on the bump side.
     */
    public BumpStopAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        // Scores cone on top row
        addCommands(new ResetGyroCommand(),
                new ChangeIntakePositionsCommand(4),
                new IntakePositionsReachedCommand(3),
                new ClawHingeCommand(),
                new ResetSteerEncodersCommand(),
                new ParallelCommandGroup(new DrivelineDrivePIDCommand(0.015, 0, 0, 0, false, true),
                        new ChangeIntakePositionsCommand(1)));

        // Drives out across the bump, leaving the Community
        addCommands(new IntakePositionsReachedCommand(3),
                new DrivelineDrivePIDCommand(0.004, 156, 180, 3, true, true));

        // Straightens TODO: check this, it might not work
        // addCommands(new DrivelineTurnPIDCommand(0, 3, true));
    }
}
