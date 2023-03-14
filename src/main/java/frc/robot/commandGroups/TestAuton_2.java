// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoHorizontalCorrectionCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.DrivelineRotationalDrivePIDCommand;
import frc.robot.commands.DrivelineTurnPIDCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.TestRotationalDriveCommand;
import frc.robot.commands.ToggleCameraModeCommand;
import frc.robot.commands.CycleCameraModeCommand;
import frc.robot.commands.VisionModeChangeCommand;
import frc.robot.commands.WaitCommand;
import frc.robot.enums.CameraMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton_2 extends SequentialCommandGroup {
  /** Creates a new Auton_2. */
  public TestAuton_2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // addCommands(new ParallelCommandGroup(new DrivelineDrivePIDCommand(36, 0, 3, true), new DrivelineTurnPIDCommand(90, 3, true)));

    // Incomplete
    // addCommands(new DrivelineRotationalDrivePIDCommand(36, 0, 0.25, 90, 3, true));

    // Test 1st: see which way this spins and determine whether the isFinished() is correct
    // Turns 90 degrees to the left with percent output
    // Before testing this at all, make sure that turning to the left returns a positive value
    // Note: I took a screenshot of the SmartDashboard the way I want it set up
    // addCommands(new DrivelineDrivePIDCommand(12, 180, 3, true));

    // Was here:

    // addCommands(new ResetGyroCommand());
    // addCommands(new TestRotationalDriveCommand(36, 180, 0.3, 90, 0.3, 5, true));

    // Test 2nd:
    // addCommands(new TestRotationalDriveCommand(36, 0, 0.25, 90, 0.25, 3, true));

    addCommands(new ToggleCameraModeCommand(CameraMode.off));
  }
}
