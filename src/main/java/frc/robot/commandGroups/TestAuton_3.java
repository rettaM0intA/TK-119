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
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.HorizontalCorrectionPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.commands.OrientWheelsCommand;
import frc.robot.commands.OrientWheelsCommand;
import frc.robot.commands.TestRotationalDriveCommand;
import frc.robot.commands.VisionAssistedGamepieceLocator; // uncomment
import frc.robot.commands.VisionModeChangeCommand; // uncomment
import frc.robot.commands.WaitCommand;
import frc.robot.enums.CameraMode;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton_3 extends SequentialCommandGroup {
  /** Creates a new TestAuton_3. */
  public TestAuton_3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // addCommands(new ResetGyroCommand(),
    // new VisionModeChangeCommand(CameraMode.gamePieceDetector),
    // new VisionAssistedGamepieceLocator(-10),
    // new ClawHingeCommand(),
    // new ChangeIntakePositionsCommand(2),
    // new IntakePositionsReachedCommand(3),
    // new ChangeClawWheelDirectionCommand(ClawWheelDirection.in),
    // new ResetGyroCommand(),
    // new TestRotationalDriveCommand(36, 0, 0.2, 0, 0, 3, true));
    // new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
    // new ChangeIntakePositionsCommand(1),
    // new IntakePositionsReachedCommand(3));

    // Current:

    // addCommands(new ResetGyroCommand(),
    //     new ChangeIntakePositionsCommand(4),
    //     new IntakePositionsReachedCommand(3),
    //     new ClawHingeCommand(),
    //     new ResetSteerEncodersCommand(),
    //     new ParallelCommandGroup(new DrivelineDrivePIDCommand(0, 0, 0, false),
    //         new ChangeIntakePositionsCommand(1)));

    // addCommands(new IntakePositionsReachedCommand(3),
    //     new DrivelineDrivePIDCommand(120, 180, 5, true),
    //     new DrivelineTurnPIDCommand(-180, 3, true)); // -175

    // addCommands(new ResetGyroCommand(),
    //     new VisionAssistedGamepieceLocator(5), // uncomment
    //     new ParallelCommandGroup(new ChangeIntakePositionsCommand(2)));

    // addCommands(new IntakePositionsReachedCommand(3),
    //     new ChangeClawWheelDirectionCommand(ClawWheelDirection.in),
    //     new ResetGyroCommand(),
    //     new DrivelineDrivePIDCommand(48, 0, 3, true),
    //     // new WaitCommand(3),
    //     new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
    //     new ChangeIntakePositionsCommand(1),
    //     new IntakePositionsReachedCommand(3));

    // addCommands(new DrivelineTurnPIDCommand(-165, 3, false));

    // addCommands(new StraightenWheelsCommand());
    // addCommands(new DrivelineDrivePIDCommand(0, 0, 1, true));
    // addCommands(new OrientWheelsCommand());
    addCommands(new VisionAssistedGamepieceLocator(0));

    // integra shins

  }
}
