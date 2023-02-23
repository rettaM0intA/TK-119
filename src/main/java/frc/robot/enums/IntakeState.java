// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.enums;

/**
 * Use for keeping track of what state the intake is in for moving the different pieces. Values are named Elevator Position, Extender Position, Hinge Position.
 */
public enum IntakeState {
    /**
     * Basic Positions. 
     */
    DownInRetract,
    DownInFloor,
    MidInStraight,
    TopOutStraight,
    PlayerInStraight
}
