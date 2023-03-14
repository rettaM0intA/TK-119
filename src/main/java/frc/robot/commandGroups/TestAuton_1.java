// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDrivelineDriveCommand;
import frc.robot.commands.AutoDrivelineTurnCommand;
import frc.robot.commands.AutoHorizontalCorrectionCommand;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.commands.VisionAssistedGamepieceLocator;
import frc.robot.commands.VisionAssistedScoreLineup;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton_1 extends SequentialCommandGroup {
  /** Auton that shoots in a ball and leaves tarmac */
  public TestAuton_1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Needs to be tested:
    // Lines up and drives forward
    // addCommands(new VisionAssistedScoreLineup(),
    // new DrivelineDrivePIDCommand(12, 0, 3, true));

    // This works! (places cube when directly in front of scoring area)
    // new ChangeIntakePositionsCommand(5),
    // new IntakePositionsReachedCommand(3),
    // new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
    // new WaitCommand(1.5),
    // new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
    // new ChangeIntakePositionsCommand(1),
    // new IntakePositionsReachedCommand(3));

  }
}
