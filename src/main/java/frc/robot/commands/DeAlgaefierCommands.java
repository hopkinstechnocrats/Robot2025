package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DeAlgefier;

public class DeAlgaefierCommands extends Command{
    private DeAlgaefierCommands(){}

    public static Command deAlgaefy(DeAlgefier deAlgaefier){
        return Commands.run(
                () -> {
                    deAlgaefier.extend();
                    deAlgaefier.dealgaefy();
                }, deAlgaefier);
    }

    public static Command stop(DeAlgefier deAlgaefier){
        return Commands.run(
                () -> {
                    deAlgaefier.retract();
                    deAlgaefier.brake();
                }, deAlgaefier);
    }
}
