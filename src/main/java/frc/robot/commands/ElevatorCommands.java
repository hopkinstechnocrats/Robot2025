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
    public static Command bottom(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
              elevator.moveToSetpoint(0);
            }
            ,

            
            elevator);
          }

          public static Command level2(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
             
              elevator.moveToSetpoint(elevatorConstants.L2Height);
            }
            ,

            
            elevator);
          }

          public static Command level3(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            
              elevator.moveToSetpoint(elevatorConstants.L3Height);
            }
            ,

            
            elevator);
          }

          public static Command level4(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
              elevator.moveToSetpoint(elevatorConstants.L4Height);
            }
            ,

            
            elevator);
          }

} 
