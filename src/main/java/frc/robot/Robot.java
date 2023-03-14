// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commandGroups.BlueBumpAcquireGamePieceAuton;
import frc.robot.commandGroups.BlueNoBumpAcquireGamePieceAuton;
import frc.robot.commandGroups.BumpStopAuton;
import frc.robot.commandGroups.ChargeStationCrossLineAuton;
import frc.robot.commandGroups.RedNoBumpAcquireGamePieceAuton;
import frc.robot.commandGroups.SchoolBoardDemoAuton;
import frc.robot.commandGroups.ScoreConeHighAuton;
import frc.robot.commandGroups.ChargeStationStopAuton;
import frc.robot.commandGroups.CycleCameraMode;
import frc.robot.commandGroups.NoBumpStopAuton;
import frc.robot.commandGroups.RedBumpAcquireGamePieceAuton;
import frc.robot.commandGroups.TestAuton_1;
import frc.robot.commandGroups.TestAuton_2;
import frc.robot.commandGroups.TestAuton_3;
import frc.robot.commandGroups.TestCommand;
import frc.robot.commands.CycleCameraModeCommand;
import frc.robot.enums.CameraMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  // public static chassisSubsystem sub_c = new chassisSubsystem();

  // public static XboxController driver = new XboxController(0);

  public static SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);
    camera.setFPS(8);

    m_robotContainer = new RobotContainer();

    // Must have:
    /**
     * Charge Station Stop: Mapped + Commented
     * Charge Station Cross Line: (!!!) M + C, incomplete
     * Red No Bump Acquire Game Piece: (!!!) M + C, needs tuning
     * Blue No Bump Acquire Game Piece: (!!!) M + C, needs tuning
     * No Bump Emergency: M + C
     * Red Bump Acquire Game Piece: (!!!) M + C, needs tuning
     * Blue Bump Acquire Game Piece: (!!!) M + C, needs tuning
     * Bump Emergency (currently identical to No Bump Emergency): M + C
     */

    // Tests
    m_chooser.setDefaultOption("Bump Emergency", new BumpStopAuton());

    // Autons
    //Charge Station Center
    m_chooser.addOption("Charge Station Stop", new ChargeStationStopAuton());
    m_chooser.addOption("Charge Station Cross Line", new ChargeStationCrossLineAuton());

    //No bump side
    m_chooser.addOption("Red NO Bump Acquire Game Piece", new RedNoBumpAcquireGamePieceAuton());
    m_chooser.addOption("Blue NO Bump Acquire Game Piece", new BlueNoBumpAcquireGamePieceAuton());

    //Bump side
    m_chooser.addOption("Red Bump Acquire Game Piece", new RedBumpAcquireGamePieceAuton());
    m_chooser.addOption("Blue Bump Acquire Game Piece", new BlueBumpAcquireGamePieceAuton());

    //Other
    // m_chooser.addOption("Score Cone", new ScoreConeHighAuton());
    m_chooser.addOption("School Board Auton", new SchoolBoardDemoAuton());
    m_chooser.addOption("Cycle Camera Mode", new CycleCameraModeCommand());

    SmartDashboard.putData("chooser", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // RobotContainer.m_chassisSubsystem.turnWheelsStraight();
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.camera.CameraMode(CameraMode.off);

    // sub_c.initSteerWheelEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // sub_c.driveTeleop(driver.getLeftTriggerAxis(),driver.getLeftTriggerAxis(),
    // driver.getRightX());

    // if(driver.getBButton()){
    // sub_c.wheelsFaceForward();
    // }else{
    // sub_c.driveTeleop(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX()
    // * 0.25);
    // }

    // sub_c.driveTeleop(0, 0, 0.25);
    // sub_c.driveTeleop(driver.getLeftY(), driver.getLeftX(), 0);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // Start the compressor.
    // RobotContainer.m_chassisSubsystem.compressor.enableDigital();
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Start the compressor.
    // RobotContainer.m_chassisSubsystem.compressor.enableDigital();
  }
}
