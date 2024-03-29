// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDrivePIDCommand;
import frc.robot.commands.IntakeFastModeCommand;
import frc.robot.commands.IntakePositionsReachedCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetSteerEncodersCommand;
import frc.robot.enums.ClawWheelDirection;
import frc.robot.commands.ChangeClawWheelDirectionCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeHighAuton extends SequentialCommandGroup {
  /** Creates a new ScoreConeHighAuton. */
  public ScoreConeHighAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyroCommand(),
        new IntakeFastModeCommand(true),
        new ChangeIntakePositionsCommand(4),
        new IntakePositionsReachedCommand(3), 
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
        new ClawHingeCommand(),
        new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
        new ChangeIntakePositionsCommand(1));
  }
}
