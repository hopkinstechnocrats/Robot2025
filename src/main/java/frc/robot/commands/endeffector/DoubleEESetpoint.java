package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class DoubleEESetpoint extends SequentialCommandGroup {
  
  public DoubleEESetpoint(EndEffectorSubsystem endeffector, double setpoint1, double setpoint2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetpointTimed(endeffector, setpoint1)
      .andThen(new SetpointTimed(endeffector, setpoint2))
    );
  }
}
