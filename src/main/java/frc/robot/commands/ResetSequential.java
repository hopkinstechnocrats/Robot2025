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
    double setpoint = elevator.getSetpoint();
    final double drop_dist = 10; // Inches to drop in order to score
    final double final_dist = 18; // Inches to drop after scoring

      addCommands(
      new ElevatorSetpoint(elevator, (setpoint - drop_dist), elevatorConstants.motorPowerResetLimit)
      .andThen(new ElevatorSetpoint(elevator, (setpoint - final_dist), elevatorConstants.motorPowerResetLimit)
      .alongWith(new EndEffectorSetpoint(endeffector, false, true)))
      .andThen(new ElevatorSetpoint(elevator, elevatorConstants.startHeight, elevatorConstants.motorPowerResetLimit)
      .alongWith(new EndEffectorSetpoint(endeffector, false, true)))
    );
  }
}
