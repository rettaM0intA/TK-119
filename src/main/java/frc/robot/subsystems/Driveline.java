// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SWERVE;

public class Driveline extends SubsystemBase {
  private final SwerveModule m_leftFront = new SwerveModule("LF", CANIDS.kDriveline_LFSteer, CANIDS.kDriveline_LFDrive,
      CANIDS.kDriveline_LFSteerEnc, InvertType.None, InvertType.InvertMotorOutput, SWERVE.kLFAbsoluteOffsetInDegrees);
  private final SwerveModule m_rightFront = new SwerveModule("RF", CANIDS.kDriveline_RFSteer, CANIDS.kDriveline_RFDrive,
      CANIDS.kDriveline_RFSteerEnc, InvertType.InvertMotorOutput, InvertType.InvertMotorOutput,
      SWERVE.kRFAbsoluteOffsetInDegrees);
  private final SwerveModule m_leftBack = new SwerveModule("LB", CANIDS.kDriveline_LBSteer, CANIDS.kDriveline_LBDrive,
      CANIDS.kDriveline_LBSteerEnc, InvertType.None, InvertType.InvertMotorOutput, SWERVE.kLBAbsoluteOffsetInDegrees);
  public final SwerveModule m_rightBack = new SwerveModule("RB", CANIDS.kDriveline_RBSteer, CANIDS.kDriveline_RBDrive,
      CANIDS.kDriveline_RBSteerEnc, InvertType.InvertMotorOutput, InvertType.InvertMotorOutput,
      SWERVE.kRBAbsoluteOffsetInDegrees);

  private final WPI_Pigeon2 m_pigeonGyro = new WPI_Pigeon2(3);
    private final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DRIVE.kDriveKinematics, m_pigeonGyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_leftFront.getModulePosition(),
          m_rightFront.getModulePosition(),
          m_leftBack.getModulePosition(),
          m_rightBack.getModulePosition()
      });
  // SwerveDriveOdometry m_odometry = new
  // SwerveDriveOdometry(DRIVE.kDriveKinematics, m_gyro.getRotation2d());
  public boolean isFieldOrientedMode = false;

  /** Creates a new Driveline. */
  public Driveline() {
    m_pigeonGyro.configFactoryDefault();
    m_pigeonGyro.reset();
    m_gyro.reset();

    m_rightFront.manualSetPID(0.107, 0, 0, 0);
  }

  public void straightenSteerMotors() {
    m_leftFront.straightenSteerMotor();
    m_rightFront.straightenSteerMotor();
    m_leftBack.straightenSteerMotor();
    m_rightBack.straightenSteerMotor();
  }

  @Override
  public void periodic() {

        // Update the robots position on the field. This is used for Autonomous.
    m_odometry.update(
        m_pigeonGyro.getRotation2d(), new SwerveModulePosition[] {
            m_leftFront.getModulePosition(),
            m_leftBack.getModulePosition(),
            m_rightFront.getModulePosition(),
            m_rightBack.getModulePosition()
        });

    SmartDashboard.putData(m_pigeonGyro);
    SmartDashboard.putData(m_gyro);
    // double dlc = (m_leftFront.getDriveCurrent()+ m_leftFront.getSteerCurrent() +
    // m_leftBack.getDriveCurrent()+ m_leftBack.getSteerCurrent() +
    // m_rightFront.getDriveCurrent()+ m_rightFront.getSteerCurrent() +
    // m_rightBack.getDriveCurrent()+ m_rightBack.getSteerCurrent());
    // SmartDashboard.putNumber("Driveline Current", dlc);
    // SmartDashboard.putNumber("Driveline LF Current",
    // m_leftFront.getDriveCurrent());
    // SmartDashboard.putNumber("Driveline LB Current",
    // m_leftBack.getDriveCurrent());
    // SmartDashboard.putNumber("Driveline RF Current",
    // m_rightFront.getDriveCurrent());
    // SmartDashboard.putNumber("Driveline RB Current",
    // m_rightBack.getDriveCurrent());
    // SmartDashboard.putData(RobotContainer.PDP);
    SmartDashboard.putNumber("FrontLeftEncoder", m_leftFront.getSteerEncAngleDeg());
    SmartDashboard.putNumber("BackLeftEncoder", m_leftBack.getSteerEncAngleDeg());
    SmartDashboard.putNumber("FrontRightEncoder", m_rightFront.getSteerEncAngleDeg());
    SmartDashboard.putNumber("BackRightEncoder", m_rightBack.getSteerEncAngleDeg());

    SmartDashboard.putNumber("Inches", getAverageDistanceInInches());

    // Gyro values
    SmartDashboard.putNumber("Pitch", getRobotPitch());
    SmartDashboard.putNumber("Roll", getRobotRoll());
    SmartDashboard.putNumber("Angle", getRobotAngle());

    SmartDashboard.putNumber("Angle2", -m_gyro.getAngle());
    SmartDashboard.putNumber("Pitch2", -m_gyro.getRoll());
    SmartDashboard.putNumber("Roll2", m_gyro.getPitch());


    SmartDashboard.putNumber("FR Absolute", m_rightFront.getAbsolutePosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_pigeonGyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_leftFront.getModulePosition(),
            m_leftBack.getModulePosition(),
            m_rightFront.getModulePosition(),
            m_rightBack.getModulePosition() },
        pose);
    m_pigeonGyro.getAngle();
  }

  /**
   * Method to drive the robot using joystick info.
   * Inputs are +/- 1 from the joystick
   * Values are converted to needed swereve module rates
   *
   * @param _xSpeed        Speed of the robot in the x direction (sideways).
   * @param _ySpeed        Speed of the robot in the y direction (forward and
   *                       backward).
   * @param _rot           Angular rate of the robot.
   * @param _fieldRelative Whether the provided x and y speeds are relative to the
   *                       field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    // Convert joystick values of +/- 1 to Meters/Sec and Rad/Sec
    // Joystick Y/X axis are reveresed here. Joystick Y is pushing forward and back.
    // The kinematics assumes X to be forward and back.
    double xSpeed = _ySpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double rot = -_rot * DRIVE.kMaxAngularRateRadPerSecond;
    // rot = 0;

    // Calculate the swerve module states
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(_fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeonGyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    // Set the desired state of each swerve module with the new calculated states.
    m_leftFront.setDesiredState(swerveModuleStates[0]);
    m_rightFront.setDesiredState(swerveModuleStates[1]);
    m_leftBack.setDesiredState(swerveModuleStates[2]);
    m_rightBack.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Autonomous Drive to inches using the PID of the TalonFX
   * The Angle must be set first so a movement does not happen before wheel is
   * fully turned
   * The best way to get the robot to drive straight is to put drive motors in a
   * velocity loop for a set time.
   * 
   * @param _xSpeed     X Speed in MPS Fwd +
   * @param _ySpeed     Y Speed in MPS Left +
   * @param _distanceIn Distances to travel in inches
   */
  public void autoDrive(double _xSpeed, double _ySpeed, double _distanceIn) {
    double xSpeed = _xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_ySpeed * DRIVE.kMaxSpeedMetersPerSecond;

    double rot = RobotContainer.driveline.getRobotAngle() * -0.15; // 0.001;
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredStateAutoMotionMagic(swerveModuleStates[0], _distanceIn);
    m_rightFront.setDesiredStateAutoMotionMagic(swerveModuleStates[1], _distanceIn);
    m_leftBack.setDesiredStateAutoMotionMagic(swerveModuleStates[2], _distanceIn);
    m_rightBack.setDesiredStateAutoMotionMagic(swerveModuleStates[3], _distanceIn);

  }

  /**
   * X and Y speed to drive at.
   * Uses robot angle to drive straight. Gyro needs to be reset if before calling
   * this
   * 
   * @param _xSpeed
   * @param _ySpeed
   */
  public void autoDrive(double _xSpeed, double _ySpeed) {
    double xSpeed = _xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_ySpeed * DRIVE.kMaxSpeedMetersPerSecond;

    double rot = RobotContainer.driveline.getRobotAngle() * -0.15;
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);

    m_leftFront.setOpenLoopRampRate(0.01);
    m_rightFront.setOpenLoopRampRate(0.01);
    m_leftBack.setOpenLoopRampRate(0.01);
    m_rightBack.setOpenLoopRampRate(0.01);

    m_leftFront.setDesiredState(swerveModuleStates[0], false);
    m_rightFront.setDesiredState(swerveModuleStates[1], false);
    m_leftBack.setDesiredState(swerveModuleStates[2], false);
    m_rightBack.setDesiredState(swerveModuleStates[3], false);
  }

  public void orientWheels(double _xComponent, double _yComponent) {
    double xSpeed = _xComponent * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_yComponent * DRIVE.kMaxSpeedMetersPerSecond;
    double rot = RobotContainer.driveline.getRobotAngle() * -0.15;
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    m_leftFront.setDesiredState(swerveModuleStates[0], true);
    m_rightFront.setDesiredState(swerveModuleStates[1], true);
    m_leftBack.setDesiredState(swerveModuleStates[2], true);
    m_rightBack.setDesiredState(swerveModuleStates[3], true);
  }

  /**
   * Autonomous Rotate wheel while driving speeds disabled
   * 
   * @param _xSpeed X Speed in MPS Fwd +
   * @param _ySpeed Y Speed in MPS Left +
   */
  public void autoRotateWheels(double _xSpeed, double _ySpeed) {
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, 0));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredState(swerveModuleStates[0], true);
    m_rightFront.setDesiredState(swerveModuleStates[1], true);
    m_leftBack.setDesiredState(swerveModuleStates[2], true);
    m_rightBack.setDesiredState(swerveModuleStates[3], true);
  }

  public void autoRotateRobot(double _rot) {
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, _rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredState(swerveModuleStates[0], false);
    m_rightFront.setDesiredState(swerveModuleStates[1], false);
    m_leftBack.setDesiredState(swerveModuleStates[2], false);
    m_rightBack.setDesiredState(swerveModuleStates[3], false);
  }

  public void resetSwerveSteerEncoders() {
    m_leftFront.resetSteerMotorCnts();
    m_rightFront.resetSteerMotorCnts();
    m_leftBack.resetSteerMotorCnts();
    m_rightBack.resetSteerMotorCnts();
  }

  public void resetSwerveDriveEncoders() {
    m_leftFront.resetDriveMotorCnts();
    m_rightFront.resetDriveMotorCnts();
    m_leftBack.resetDriveMotorCnts();
    m_rightBack.resetDriveMotorCnts();
  }

  public double getAverageDistanceInInches() {
    double dis = (Math.abs(m_leftFront.getDriveMotorCnts()) + Math.abs(m_leftBack.getDriveMotorCnts())
        + Math.abs(m_rightBack.getDriveMotorCnts()) + Math.abs(m_rightFront.getDriveMotorCnts())) / 4.0;
    return dis / SWERVE.kDriveCntsPerInch;
  }

  public double getAverageVelocity() {
    double vel = (m_leftFront.getDriveVelocityInNativeUnits() + m_leftBack.getDriveVelocityInNativeUnits()
        + m_rightBack.getDriveVelocityInNativeUnits() + m_rightFront.getDriveVelocityInNativeUnits()) / 4.0;
    return vel;
  }

  // When using Pigeon 
  public double getRobotAngle() {
    return -m_pigeonGyro.getAngle();
  }

  // When using NavX
  // public double getRobotAngle() {
  //   return -m_gyro.getAngle();
  // }

  public double getRobotRoll() {
    return m_pigeonGyro.getPitch();
  }

  // When using Pigeon
  public double getRobotPitch() {
    return -m_pigeonGyro.getRoll();
  }

  // When using NavX
  // public double getRobotPitch() {
  //   return -m_gyro.getRoll();
  //   }

  public void resetGyro() {
    m_pigeonGyro.reset();
  }

  public void setFieldOrientedMode() {
    isFieldOrientedMode = !isFieldOrientedMode;
  }

  public boolean getFieldOrientedModeActive() {
    return isFieldOrientedMode;
  }

  public double getCompassHeading() {

    return m_pigeonGyro.getCompassHeading();
  }

  public WPI_Pigeon2 getGyro() {
    return m_pigeonGyro;
  }

  public void toggleFieldOrientedMode() {
    isFieldOrientedMode = !isFieldOrientedMode;
  }

  public void drive(double _xSpeed, double _ySpeed, double _rot, int RotationOffset) {
    // Convert joystick values of +/- 1 to Meters/Sec and Rad/Sec
    // Joystick Y/X axis are reveresed here. Joystick Y is pushing forward and back.
    // The kinematics assumes X to be forward and back.
    double xSpeed = _ySpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double rot = -_rot * DRIVE.kMaxAngularRateRadPerSecond;
    // rot = 0;

    // Calculate the swerve module states
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(true
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            new Rotation2d(m_pigeonGyro.getRotation2d().getRadians() + (RotationOffset * (Math.PI / 180))))
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    // Set the desired state of each swerve module with the new calculated states.
    m_leftFront.setDesiredState(swerveModuleStates[0]);
    m_rightFront.setDesiredState(swerveModuleStates[1]);
    m_leftBack.setDesiredState(swerveModuleStates[2]);
    m_rightBack.setDesiredState(swerveModuleStates[3]);
  }
}
