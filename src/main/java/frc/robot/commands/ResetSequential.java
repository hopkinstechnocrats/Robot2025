package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.endeffector.EndEffectorSetpoint;

public class ResetSequential extends SequentialCommandGroup {
  
  public ResetSequential (ElevatorSubsystem elevator, EndEffectorSubsystem endeffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

      addCommands(
      new ElevatorSetpoint(elevator, elevator.getSetpoint() * 0.7, elevatorConstants.motorPowerResetLimit)
      .andThen(new ElevatorSetpoint(elevator, elevatorConstants.startHeight, elevatorConstants.motorPowerResetLimit))
      .alongWith(new EndEffectorSetpoint(endeffector, true, true, true))
    );
  }
}
