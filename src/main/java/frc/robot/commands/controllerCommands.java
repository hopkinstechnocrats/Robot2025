

// public class controllerCommands {
//     private controllerCommands(){};

//       public static Command getTargetHeading(ControllerSubsystem getTargetHeading, CommandXboxController controller){
//       return Commands.run(
//         () ->   {
//             ControllerSubsystem.getTargetHeading(controller);
//         },
//          controller);
//     }
// }


package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ControllerSubsystem;


public class controllerCommands extends Command{
    private controllerCommands(){}

    public static Command getTargetHeading(ControllerSubsystem controllerSubsystem, CommandXboxController controller){
        return Commands.run(
            () -> {
                controllerSubsystem.getTargetHeading(controller);
            }, controllerSubsystem);
    }


} 
