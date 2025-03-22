package frc.robot.commands.elevator;

import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetSetpoint extends Command {

    private  ElevatorSubsystem m_elevator;
    private Timer m_timer;

    private static final double kMoveSpeed = -0.4; //Adjust as needed
    private static final double kMoveDuration = 1.0; //Adjust as needed

    public void ElevatorTimed(ElevatorSubsystem elevator, double t) {
        m_elevator = elevator;
        m_timer = new Timer();
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        //m_elevator.changeSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        //m_elevator.ArcadeDrive(0, 0);
        System.out.println("DriveForwardCmd ended!");
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= kMoveDuration;
    }   
}