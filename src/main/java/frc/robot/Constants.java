// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OI {
        public static final double kDeadband = 0.12;
    }
	
    public static final class COMM {
        public static final int kTimeoutMs = 30;
    }

    public static final class CANIDS {
        public static final int kDriveline_LFSteer = 27;
        public static final int kDriveline_LFDrive = 28;
        public static final int kDriveline_LFSteerEnc = 8;
        public static final int kDriveline_LBSteer = 21;
        public static final int kDriveline_LBDrive = 22;
        public static final int kDriveline_LBSteerEnc = 2;
        public static final int kDriveline_RFSteer = 25;
        public static final int kDriveline_RFDrive = 26;
        public static final int kDriveline_RFSteerEnc = 6;
        public static final int kDriveline_RBSteer = 23;
        public static final int kDriveline_RBDrive = 24;
        public static final int kDriveline_RBSteerEnc = 4;

	}
	
    public static final class DRIVE {
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.56;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.56;
        public static final double kWheelTrackBaseCircumference = 2 * Math.PI
                * (Math.sqrt(Math.pow((kTrackWidth / 2), 2) + Math.pow((kWheelBase / 2), 2)));
        // Converts units/100ms to wheel m/sec
        // Units/100ms max measured = 22,036
        public static final double kDriveVelRatio = 503.6815;
        public static final double kMaxDriveVelUnitsPer100ms = 21777.0;
        public static final double kMaxSpeedMetersPerSecond = 4.324;
        public static final double kMaxAngularRateRadPerSecond = 4.324;//((1 / kWheelTrackBaseCircumference / 2 * Math.PI)
               // * kMaxSpeedMetersPerSecond) * 1;
        // frontLeft, frontRight, rearLeft, rearRight
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    }

    public static final class SWERVE {

        public static final double kDriveMotEncoderCountsPerRev = 2048.0;
        public static final double kDriveRatio = 7.85;
        public static final double kWheelDiameter = 4.0;
        public static final double kDriveCntsPerInch = kDriveMotEncoderCountsPerRev * kDriveRatio / (Math.PI * kWheelDiameter);
        public static final int kDriveAutoMotionMagicSlotIdx = 0;
        //https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        // Starting kP of 0.01. Adjust higher to get closer to 10888 or lower to stop oscilations.
        public static final double kDriveMotionMagic_kP = 0.0;
        // Start with small value
        public static final double kDriveMotionMagic_kI = 0.0;
        // Starting kD is typically 10 x kP
        public static final double kDriveMotionMagic_kD = 0.0;
        // kF 50% speed. (0.50*1023) / (kMaxDriveVelUnitsPer100ms * 0.5) = 0.047
        public static final double kDriveMotionMagic_kF = 0.5; 
        // Set Cruise Velocit to 1/2 of max like kF. 
        public static final double kDriveMotionMagic_CruiseVel = 10888;
        // Set Accel to 1/2 of Max for 1 second ramp up.
        public static final double kDriveMotionMagic_Accel = 500;
        public static final int kDriveMotionMagic_Smoothing = 1;

        public static final double kSteerMotEncoderCountsPerRev = 2048.0;
        public static final double kSteerRatio = 15.43;
        public static final double kSteerMotCntsPerWheelDeg = (kSteerMotEncoderCountsPerRev * kSteerRatio) / 360;
        public static final double kSteerMotCountsPerWheelRadian = (kSteerMotEncoderCountsPerRev / (2 * Math.PI)) * kSteerRatio;
        public static final double kSteerEncoderCountsPerRev = 4096.0;
        public static final double kSteerCountsPerRadian = kSteerEncoderCountsPerRev / 2 * Math.PI;

        //REAL absolute offsets
        public static final double kLFAbsoluteOffsetInDegrees = 71.2;
        public static final double kLBAbsoluteOffsetInDegrees = 143.1;
        public static final double kRFAbsoluteOffsetInDegrees = 162.3;
        public static final double kRBAbsoluteOffsetInDegrees = 114.9;
    }


	public static final double kDirectionalDeadzone = 0.02;
    public static final double kFwd_MetersPerSecPerNom = 1;
	public static final double kStrafe_MetersPerSecPerNom = 0;
	public static final double kRotation_RadiansPerSecPerNom = 0;
	public static final double kMaxMetersPerSecWheelSpeeds = 0;
	public static final double kCntsPerDeg = 12.8;
	public static final double kMetersPerSecPerNom = 0;


	// Straight ahead offset of wheel in degrees
	public static final double kChassisFLAbsOffsetDeg = 26.316;
	public static final double kChassisFRAbsOffsetDeg = 27.72;
	public static final double kChassisBLAbsOffsetDeg = 90.36;
	public static final double kChassisBRAbsOffsetDeg = 21.42;
	public static final double kChassisSteerMotorGearRatio = 1; // 37.5

	// Theoretical based off the Max speed of the motor of 5676 RPM
	public static final double kChassisMaxMetersPerSec = 5.67;
	public static final double kChassisMaxRadiansPerSec = 18.59;

	public static final double kChassisEstimatedRotationsToInches = 2048*7.45/12; //motor counts, conversion to wheel turns, amount of inches per turn.

	// The gear ratios for the serve turning motor
	public static final double kChassisNeoToGearbox = 80;
	public static final double kChassisGearboxToOutputGear = 48;
	public static final double kChassisOutputDriveGearToInputGear = 40;
	public static final double kChassisNeoMotorRotationPerWheelRotation = kChassisNeoToGearbox / kChassisGearboxToOutputGear * kChassisOutputDriveGearToInputGear;
	public static final double kChassisNeoMotorRotationtoRadians = kChassisNeoMotorRotationPerWheelRotation * Math.PI;
	
	public static final double kChassisCANCoderOffsetfL = 86.484375;
	public static final double kChassisCANCoderOffsetfR = 227.8125;
	public static final double kChassisCANCoderOffsetbR = 171.826171875;
	public static final double kChassisCANCoderOffsetbL = 142.20703125;

	public static final double kChassisFalconToWheelRatio = 2048 * 3 * (72/14);		//0.64814;
	public static final double kChassisDegreetoMotor = 360 / kChassisFalconToWheelRatio;

	//This is used to lower the speed of the drive motors to reasonable values
	public static final double kChassisMotorSpeedLower = 5.67;
	
	// 2048 : 8 > 24 : 14 > 72 : 360 New Chassis Gearbox
	
	public static final double kChassisSwerveOutputDegreeToNeoRotation = 360 / kChassisNeoMotorRotationPerWheelRotation;	//5.4000054000054

	public static final double kChassisAbsoluteZerofL = 3.118;	//Absolute reads 4.118	lowered for conversion
	public static final double kChassisAbsoluteZerofR = 1.97;	//Absolute reads 2.97	lowered for conversion
	public static final double kChassisAbsoluteZerobL = 2.52;	//Absolute reads 2.52	unchanged for conversion
	public static final double kChassisAbsoluteZerobR = 0.81;	//Absolute reads 0.81	unchanged for conversion
	public static final int kChassisAbsoluteToDegreeConversion = 90;

}
