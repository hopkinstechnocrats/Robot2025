package frc.robot.commands;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;


public class ClimbCommands extends Command{
    private ClimbCommands(){}

    public static Command brake(Climber climbSubsystem){
        return Commands.run(() -> {
            climbSubsystem.brake();
            climbSubsystem.brakeVictor();
        }, climbSubsystem);
    }

    public static Command extendClimber(Climber climbSubsystem){
        return Commands.run(() -> {
            climbSubsystem.extend();
        }, climbSubsystem);
    }

    public static Command retractClimber(Climber climbSubsystem){
        return Commands.run(() -> {
            climbSubsystem.retract();
        }, climbSubsystem);
    }

    public static Command spinVictor(Climber climbSubsystem){
        return Commands.runOnce(() -> {
            climbSubsystem.spinVictor();
        }, climbSubsystem);
    }

   
}
