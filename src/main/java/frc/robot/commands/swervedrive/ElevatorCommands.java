package frc.robot.commands.swervedrive;

import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    private ElevatorCommands() {}

    public static Command elevatorUp(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorUp();
            },
            elevator);
      }
    
      public static Command elevatorDown(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorDown();
            },
            elevator);
      }

      public static Command stop(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.stopElevator();
            },
            elevator);
      }
    }