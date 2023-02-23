// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveStraight;
// import frc.robot.commands.AutoIntakeCommand;
// import frc.robot.commands.IntakeSpinCommand;
// import frc.robot.commands.AutoChassisMovePid;
// import frc.robot.commands.AutoChassisSpinPID;
// import frc.robot.commands.AutoIntakeCommand;
// import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_1 extends SequentialCommandGroup {
  /** Auton that shoots in a ball and leaves tarmac*/
  public Auton_1() {   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    RobotContainer.driveline.resetSwerveDriveEncoders();

    // addCommands(new ResetGyroCommand(), new AutoDriveStraight(0, -0.3, -60));

    // addCommands(new IntakeSpinCommand(false));

    //Resets Gyro
    addCommands(new ResetGyroCommand());
    //Spit first ball
    // addCommands(new AutoIntakeCommand(false, 1, false));
    //Move backwards
    addCommands(new AutoDriveStraight(-.02, -0.3, 80));
  }
}