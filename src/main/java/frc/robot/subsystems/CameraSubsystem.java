// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;

public class CameraSubsystem extends SubsystemBase {

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public NetworkTableEntry tx = table.getEntry("tx");
  public NetworkTableEntry ty = table.getEntry("ty");
  public NetworkTableEntry ta = table.getEntry("ta");
  

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {

    table.getEntry("stream").setDouble(0);
    CameraMode(false);

  }

  /**
   * 
   * @param mode set true to make the camera visible. Set false to have it on detect mode.
   */
  public void CameraMode(boolean mode){

    //pipeline 1 is used for finding april tags
    //pipeline 0 is used for finding cones

    if(mode){
      //Set the search mode to 0 (Retroflective mode)
      table.getEntry("pileline").setDouble(0);


      //Turn on the lights!
      table.getEntry("ledMode").setInteger(1);
    }else{
      //Set the search mode to 1 (apriltags)
      table.getEntry("pileline").setDouble(1);

      //set the cam mode to 0 to disable the camera visual feed. Allows finding
      table.getEntry("camMode").setInteger(0);

      //Turn off the lights!
      table.getEntry("ledMode").setInteger(0);
    }

  }

  public boolean TargetLocated(){

    //tv returns the amount of visable targets. This simply varifies that there are more than 0 of them.
    NetworkTableEntry tv = table.getEntry("tv");
    SmartDashboard.putNumber("tv", tv.getDouble(0));

    if(tv.getInteger(0) < 1){
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double pipeline = table.getEntry("pipeline").getDouble(500);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightType", pipeline);
  }
}
