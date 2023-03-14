// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangeClawWheelDirectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ToggleCameraModeCommand;
import frc.robot.commands.VisionAssistedGamepieceLocator;
import frc.robot.enums.CameraMode;
import frc.robot.enums.ClawWheelDirection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SchoolBoardDemoAuton extends SequentialCommandGroup {
  /** Creates a new SchoolBoardCommand. */
  public SchoolBoardDemoAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Adjusts with vision
    addCommands(new ResetGyroCommand(),
        new ToggleCameraModeCommand(CameraMode.gamePieceDetector),
        new VisionAssistedGamepieceLocator(0),
        new ParallelCommandGroup(new ChangeIntakePositionsCommand(2)));

    // Drives forward and picks up the cube
    addCommands(new IntakePositionsReachedCommand(3),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.in),
        new ResetGyroCommand(),
        new DrivelineDrivePIDCommand(0.015, 48, 0, 3, true),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
        // Turns off the Limelight
        new ParallelCommandGroup(new ChangeIntakePositionsCommand(1), new ToggleCameraModeCommand(CameraMode.off)));

    addCommands(new IntakePositionsReachedCommand(3));
  }
}
