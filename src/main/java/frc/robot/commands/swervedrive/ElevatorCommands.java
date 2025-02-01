package frc.robot.commands.swervedrive;

import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
import frc.robot.Constants;

public class MechanismCommands {
    private MechanismCommands() {}

    public static Command elevatorUp(Elevator elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorUp();
            },
            elevator);
      }
    
      public static Command climbDown(Elevator elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorDown();
            },
            elevator);
      }
    }