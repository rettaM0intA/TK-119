// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CameraDefaultCommand;
import frc.robot.commands.ChangeIntakePositionsCommand;
import frc.robot.commands.ClawDefaultCommand;
import frc.robot.commands.ClawHingeCommand;
import frc.robot.commands.DrivelineDefaultCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.ExtenderDefaultCommand;
import frc.robot.commands.HingeDefaultCommand;
import frc.robot.commands.PneumaticSwitch;
import frc.robot.commands.ToggleFieldOrientedModeCommand;
import frc.robot.enums.ElevatorPosition;
import frc.robot.enums.ExtenderPosition;
import frc.robot.enums.HingePosition;
import frc.robot.subsystems.CameraSubsystem; // uncomment
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Driveline;
import frc.robot.subsystems.ElevatorsSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
// import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.HingeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  ActiveMode activeMode = ActiveMode.test;

  // // The robot's subsystems and commands are defined here...
  // public static chassisSubsystem m_chassisSubsystem = new chassisSubsystem();

  public static final Driveline driveline = new Driveline();
  private DrivelineDefaultCommand drivelineDefaultCommand = new DrivelineDefaultCommand(driveline);
  
  public static CameraSubsystem camera = new CameraSubsystem(); // uncomment
  public static CameraDefaultCommand cameraDefaultCommand = new CameraDefaultCommand();

  // public static GrabberSubsystem grabber = new GrabberSubsystem();

  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public static ControllerInControl gamepadDriver = ControllerInControl.flightStick;

  public static ElevatorsSubsystem elevator = new ElevatorsSubsystem();
  private ElevatorDefaultCommand elevatorDefaultCommand = new ElevatorDefaultCommand();

  public static ExtenderSubsystem extender = new ExtenderSubsystem();
  public ExtenderDefaultCommand extenderDefaultCommand = new ExtenderDefaultCommand();
  public static ExtenderPosition extendPos = ExtenderPosition.Retracted;

  public static HingeSubsystem hinge = new HingeSubsystem();
  private HingeDefaultCommand hingeDefaultCommand = new HingeDefaultCommand();
  public static HingePosition hingePos = HingePosition.Retracted;

  public static ClawSubsystem claw = new ClawSubsystem();
  private ClawDefaultCommand clawDefaultCommand = new ClawDefaultCommand();
  public static boolean clawClosed = true;

  public static ElevatorPosition elevatPos = ElevatorPosition.Floor;

  public static boolean fullSpeed = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    LiveWindow.disableAllTelemetry();

    driveline.setDefaultCommand(drivelineDefaultCommand);

    // camera.setDefaultCommand(visionAssistedItemPlacement);
    elevator.setDefaultCommand(elevatorDefaultCommand);
    extender.setDefaultCommand(extenderDefaultCommand);
    hinge.setDefaultCommand(hingeDefaultCommand);
    claw.setDefaultCommand(clawDefaultCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    JoystickButton DrivelineFieldOrientedModeActive;

    // DrivelineFieldOrientedModeActive = new JoystickButton(driver, 6);

    JoystickButton pneumaticsSwitch = new JoystickButton(driver, 2);
    pneumaticsSwitch.onTrue(new PneumaticSwitch());
    
    JoystickButton inDownRetractButton = new JoystickButton(operator, 1);
    inDownRetractButton.onTrue(new ChangeIntakePositionsCommand(1));
      
    JoystickButton inDownFloorButton = new JoystickButton(operator, 2);
    inDownFloorButton.onTrue(new ChangeIntakePositionsCommand(2));

    JoystickButton inMidStraightButton = new JoystickButton(operator, 3);
    inMidStraightButton.onTrue(new ChangeIntakePositionsCommand(3));

    JoystickButton outHighStraighButton = new JoystickButton(operator, 4);
    outHighStraighButton.onTrue(new ChangeIntakePositionsCommand(4));

    JoystickButton clawHingeButton = new JoystickButton(operator, 5);
    clawHingeButton.onTrue(new ClawHingeCommand());

    JoystickButton inCollectStraightButton = new JoystickButton(operator, 6);
    inCollectStraightButton.onTrue(new ChangeIntakePositionsCommand(5));

    JoystickButton clawHingeTemp = new JoystickButton(driver, 5);
    clawHingeTemp.onTrue(new ClawHingeCommand());

    JoystickButton toggleFieldOrientedMode = new JoystickButton(driver, 4);
    toggleFieldOrientedMode.onTrue(new ToggleFieldOrientedModeCommand());
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new TestCommand();
    return Robot.m_chooser.getSelected();
  }
}
