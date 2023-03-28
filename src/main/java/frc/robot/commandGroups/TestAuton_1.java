// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commandGroups;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoHorizontalCorrectionCommand;
// import frc.robot.commands.CrossChargeStationCommand;
// import frc.robot.commands.DrivelineDrivePIDCommand;
// import frc.robot.commands.DrivelineTurnPIDCommand;
// import frc.robot.commands.ToggleCameraModeCommand;
// import frc.robot.enums.CameraMode;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TestAuton_1 extends SequentialCommandGroup {
//   /** Auton that shoots in a ball and leaves tarmac */
//   public TestAuton_1() {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());

//     addCommands(new ToggleCameraModeCommand(CameraMode.off));

//     // Drives across the Charge Station
//     addCommands(new DrivelineDrivePIDCommand(0.0175, 48, 180, 3, true, true),
//         // Time may still need to be slightly less on stopping at the end
//         // TODO: manually time how long it takes to get halfway across and all the way
//         new CrossChargeStationCommand(5, true));

//     addCommands(new DrivelineTurnPIDCommand(0, 3, true));

//     // addCommands(new DrivelineDrivePIDCommandCodyVer(0.005, 48, 180, 5, true,
//     // true));

//     // This works for approaching the Charge Station with the front of the robot:
//     addCommands(new DrivelineDrivePIDCommand(0.0175, 56, 0, 3, false, true),
//         new AutoHorizontalCorrectionCommand(0.01, 5, true));

//     // This works! (places cube when directly in front of scoring area)
//     // new ChangeIntakePositionsCommand(5),
//     // new IntakePositionsReachedCommand(3),
//     // new ChangeClawWheelDirectionCommand(ClawWheelDirection.out),
//     // new WaitCommand(1.5),
//     // new ChangeClawWheelDirectionCommand(ClawWheelDirection.stop),
//     // new ChangeIntakePositionsCommand(1),
//     // new IntakePositionsReachedCommand(3));

//   }
// }
