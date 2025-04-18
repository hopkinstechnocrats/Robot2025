package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorSetpoint extends Command {

    private final ElevatorSubsystem m_elevator;
    private double m_setpoint;
    private double m_speed;

    public ElevatorSetpoint(ElevatorSubsystem elevator, double setpoint, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_elevator = elevator;
        m_setpoint = setpoint;
        m_speed = speed;
        addRequirements(m_elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.changeSetpoint(m_setpoint, m_speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
