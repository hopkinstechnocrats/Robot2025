package frc.robot.commands.endeffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;

public class SetpointTimed extends Command {

    private final EndEffectorSubsystem m_endeffector;
    private double m_setpoint;

    public SetpointTimed(EndEffectorSubsystem endeffector, double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_endeffector = endeffector;
        m_setpoint = setpoint;
        addRequirements(m_endeffector);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("EE Change Setpoint Command Started");
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
    System.out.println("EE Change Setpoint Command Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("EE Are we at setpoint? " + m_endeffector.atSetpoint());
    return m_endeffector.atSetpoint();
  }
}
