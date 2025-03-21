package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands extends Command{
    private ElevatorCommands(){}

    public static Command up(ElevatorSubsystem elevatorSubsystem){
        return Commands.run(
            () -> {
                elevatorSubsystem.up();
            }, elevatorSubsystem);
    }

    public static Command down(ElevatorSubsystem elevatorSubsystem){
        return Commands.run(
            () -> {
                elevatorSubsystem.down();;
            }, elevatorSubsystem);
    }

    public static Command brake(ElevatorSubsystem elevatorSubsystem){
        return Commands.run(
            () -> {
                elevatorSubsystem.brake();
            }, elevatorSubsystem);
    }


    public static Command setpointMove(ElevatorSubsystem elevator){
        return Commands.run(
          () ->   {
              elevator.moveToSetpoint();
          },
            elevator);
    }

    public static Command setSetpoint(ElevatorSubsystem elevator, Double setpoint){
      return Commands.runOnce(
        () ->   {
            elevator.changeSetpoint(setpoint);
        },
         elevator);
    }

    public static Command setSetpointAndMove(ElevatorSubsystem elevator, Double setpoint){
        return Commands.runOnce(
          () ->   {
              elevator.changeSetpoint(setpoint);
              elevator.moveToSetpoint();
          },
           elevator);
      }

} 
