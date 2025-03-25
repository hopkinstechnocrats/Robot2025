package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class DoubleSetpoint extends SequentialCommandGroup {
  
  public DoubleSetpoint(ElevatorSubsystem elevator, double setpoint1, double setpoint2, double wait) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChangeSetpointCommand(elevator, setpoint1),
      new WaitCommand(wait),
      new ChangeSetpointCommand(elevator, setpoint2)
    );
  }
}
