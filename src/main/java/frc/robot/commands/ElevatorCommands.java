package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public static Command level1(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            while (elevator.encoderRotation() != 1.5) 
              elevator.level1();
            }
            ,

            
            elevator);
          }

          public static Command level2(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            if (elevator.encoderRotation() != 2.5) 
              elevator.level2();
            }
            ,

            
            elevator);
          }

          public static Command level3(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            if (elevator.encoderRotation() != 3.5) 
              elevator.level3();
            }
            ,

            
            elevator);
          }

          public static Command level4(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            if (elevator.encoderRotation() != 4.5) 
              elevator.level4();
            }
            ,

            
            elevator);
          }

} 
