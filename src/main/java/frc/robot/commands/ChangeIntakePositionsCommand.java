// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.enums.ElevatorPosition;
import frc.robot.enums.ExtenderPosition;
import frc.robot.enums.HingePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeIntakePositionsCommand extends InstantCommand {

  int desiredState;

  /**
   * Changes the chosen positions of the intake pieces to the positions that match the entered number.
   * @param _desiredStateNumber set to the number that matches the state you want
   * 1: everything down and in. In Down Retracted
   * 2: claw facing the floor. In Down Floor
   * 3: preset for mid cone and high cube. In Mid Straight
   * 4: preset for High cone. Out High Angled
   * 5: preset for the human player shelf. In Shelf Straight
   */
  public ChangeIntakePositionsCommand(int _desiredStateNumber) {

    desiredState = _desiredStateNumber;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(desiredState){
      case 1:
        RobotContainer.extendPos = ExtenderPosition.Retracted;
        RobotContainer.elevatPos = ElevatorPosition.Floor;
        RobotContainer.hingePos = HingePosition.Retracted;
        break;
        // Obtain game pieces from the floor
      case 2:
        RobotContainer.extendPos = ExtenderPosition.Retracted;
        RobotContainer.elevatPos = ElevatorPosition.Floor;
        RobotContainer.hingePos = HingePosition.Floor;
        break;
      case 3:
        RobotContainer.extendPos = ExtenderPosition.Retracted;
        RobotContainer.elevatPos = ElevatorPosition.Mid;
        RobotContainer.hingePos = HingePosition.Straight;
        break;
        // Score in the upper area
      case 4:
        RobotContainer.extendPos = ExtenderPosition.Extended;
        RobotContainer.elevatPos = ElevatorPosition.Top;
        RobotContainer.hingePos = HingePosition.HighGoal;
        break;
      case 5:
        RobotContainer.extendPos = ExtenderPosition.Retracted;
        RobotContainer.elevatPos = ElevatorPosition.Shelf;
        RobotContainer.hingePos = HingePosition.Straight;

    }
  }
}
