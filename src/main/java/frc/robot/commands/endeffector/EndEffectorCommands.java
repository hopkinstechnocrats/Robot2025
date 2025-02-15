package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.EndEffector;

public class EndEffectorCommands extends Command {
    public static Command moveLeft(EndEffector endEffector){
        return Commands.run(
                () -> {
                    endEffector.moveLeft();
                },
            endEffector);
    }

    public static Command moveRight(EndEffector endEffector){
        return Commands.run(
                () -> {
                    endEffector.moveRight();
                },
            endEffector);
    }


    public static Command brake(EndEffector endEffector){
        return Commands.run(
                () -> {
                    endEffector.brake();
                }, endEffector);
    }
} 
