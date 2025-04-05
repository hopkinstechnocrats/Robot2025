package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.endeffector.EndEffectorSetpoint;

public class ResetSequential extends SequentialCommandGroup {
  
  public ResetSequential (ElevatorSubsystem elevator, EndEffectorSubsystem endeffector, double setpoint, double score, double reset) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double m_score = score; // Inches to drop in order to score
    final double m_reset = reset; // Inches to drop after scoring

    addCommands(
      new ElevatorSetpoint(elevator, (setpoint - score), elevatorConstants.motorPowerResetLimit)
      .andThen(new ElevatorSetpoint(elevator, (setpoint - reset), elevatorConstants.motorPowerResetLimit)
      .alongWith(new EndEffectorSetpoint(endeffector, false, true)))
      /*.andThen(new ElevatorSetpoint(elevator, elevatorConstants.startHeight, elevatorConstants.motorPowerResetLimit)
      .alongWith(new EndEffectorSetpoint(endeffector, false, true)))*/
    );
  }
}
