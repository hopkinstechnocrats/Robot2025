package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommands extends Command {
    public static Command changeSetpointCommand(EndEffectorSubsystem endEffector, double setpoint){
        return Commands.runOnce(
                () -> {
                    endEffector.changeSetpoint(setpoint);
                },
            endEffector);
    }

    public static Command moveToSetpointCommand(EndEffectorSubsystem endEffector){
        return Commands.run(
                () -> {
                    endEffector.moveToSetpoint();
                },
            endEffector);
    }
} 
