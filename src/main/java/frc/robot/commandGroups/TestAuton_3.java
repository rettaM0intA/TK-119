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

    addCommands(new VisionAssistedGamepieceLocator());

    // integra shins
    // Emma May = MMA

  }
}
