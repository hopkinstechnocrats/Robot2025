package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;


public class ClimbCommands extends Command{
    private ClimbCommands(){}

    public static Command brake(Climber climbSubsystem){
        return Commands.run(() -> {
            climbSubsystem.brake();
        }, climbSubsystem);
    }

     public static Command climb(Climber climbSubsystem){
        return Commands.run(() -> {
            climbSubsystem.climb();
        }, climbSubsystem);
    }

   
}
