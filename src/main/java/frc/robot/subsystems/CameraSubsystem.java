// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.CameraMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraSubsystem extends SubsystemBase {

  CameraMode currentCameraMode;

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /**
   * Horizontal offset
   */
  public NetworkTableEntry tx = table.getEntry("tx");

  /**
   * Vertical offset
   */
  public NetworkTableEntry ty = table.getEntry("ty");

  /**
   * Visual size
   */
  public NetworkTableEntry ta = table.getEntry("ta");

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {

    table.getEntry("stream").setDouble(0);
    CameraMode(CameraMode.off);

  }

  /**
   * 
   * @param mode set true to make the camera visible. Set false to have it on
   *             detect mode.
   */
  public void CameraMode(CameraMode mode) {
    currentCameraMode = mode;
    // pipeline 1 is used for finding april tags
    // pipeline 0 is used for finding cones
    switch (mode) {
      case off:
        // Set the search mode to 0 (Gamepieces)
        table.getEntry("pipeline").setNumber(0);

        table.getEntry("camMode").setNumber(0);

        // Turn off the lights!
        table.getEntry("ledMode").setNumber(1); // 1
        break;

      case aprilTag:
        // Set the search mode to 1 (apriltags)
        table.getEntry("pipeline").setNumber(1);

        // set the cam mode to 0 to disable the camera visual feed. Allows finding
        table.getEntry("camMode").setNumber(0);

        // Turn off the lights!
        table.getEntry("ledMode").setNumber(1); // 1
        break;

      case gamePieceDetector:
        // Set the search mode to 5 (GamePeices)
        table.getEntry("pipeline").setNumber(0);

        // set the cam mode to 0 to disable the camera visual feed. Allows finding
        table.getEntry("camMode").setNumber(0);

        // Turn on the lights!
        table.getEntry("ledMode").setNumber(3); // 3
        break;
    }

  }

  public CameraMode getCurrentCameraMode() {
    return currentCameraMode;
  }

  public boolean TargetLocated() {

    // tv returns the amount of visable targets. This simply varifies that there are
    // more than 0 of them.
    NetworkTableEntry tv = table.getEntry("tv");
    // SmartDashboard.putNumber("tv", tv.getDouble(0));

    if (tv.getInteger(0) < 1) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read values periodically
    // double x = tx.getDouble(500);
    // double y = ty.getDouble(500);
    // double area = ta.getDouble(500);
    double pipeline = table.getEntry("pipeline").getInteger(500);

    // post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightType", pipeline);
  }
}
