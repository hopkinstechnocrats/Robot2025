package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.endeffector.EndEffectorSetpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class ScoreSequential extends SequentialCommandGroup {
  
  public ScoreSequential(ElevatorSubsystem elevator, EndEffectorSubsystem endeffector, double setpoint, boolean left, boolean level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(
      new ElevatorSetpoint(elevator, setpoint)
      .andThen(new EndEffectorSetpoint(endeffector, left, level, false))
    );
  }
}