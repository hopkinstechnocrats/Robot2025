package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.endeffector.EndEffectorSetpoint;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class ScoreSequential extends SequentialCommandGroup {
  
  public ScoreSequential(ElevatorSubsystem elevator, EndEffectorSubsystem endeffector, double setpoint, boolean level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double score_offset = 20;
    addCommands(
      new ElevatorSetpoint(elevator, (setpoint - score_offset), elevatorConstants.motorPowerLimit)
      .andThen(new ElevatorSetpoint(elevator, setpoint, elevatorConstants.motorPowerLimit)
      .alongWith(new EndEffectorSetpoint(endeffector, level, false)))
    );
  }
}