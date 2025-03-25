package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants;

public class ChangeSetpointCommand extends Command {

    private final ElevatorSubsystem m_elevator;
    private double m_setpoint;

    public ChangeSetpointCommand(ElevatorSubsystem elevator, double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_elevator = elevator;
        m_setpoint = setpoint;
        addRequirements(m_elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Change Setpoint Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.changeSetpoint(m_setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Change Setpoint Command Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
