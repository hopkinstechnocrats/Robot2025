package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.endEffectorConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorSetpoint extends Command {

    private final EndEffectorSubsystem m_endeffector;
    private double m_setpoint;

    public EndEffectorSetpoint(EndEffectorSubsystem endeffector, boolean level, boolean storage) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_endeffector = endeffector;
        if (storage) {
          m_setpoint = endEffectorConstants.Stowage;
        } else if (level) m_setpoint = endEffectorConstants.LeftScoreL4; else m_setpoint = endEffectorConstants.LeftScore; 
        addRequirements(m_endeffector);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_endeffector.changeSetpoint(m_setpoint);
    m_endeffector.moveToSetpoint();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endeffector.atSetpoint();
  }
}
