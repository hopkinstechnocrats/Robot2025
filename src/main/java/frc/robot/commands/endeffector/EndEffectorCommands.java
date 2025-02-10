package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.EndEffector;

public class EndEffectorCommands extends Command {
    public static Command move1(EndEffector endEffector){
        return Commands.run(
                () -> {
                    endEffector.moveToSetpoint(0.1);
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
