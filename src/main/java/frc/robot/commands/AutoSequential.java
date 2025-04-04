package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class AutoSequential extends Command{

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final EndEffectorSubsystem m_endEffectorSubsystem;
    private final double m_EVSetpoint;
    private final double m_EESetpoint;

    public AutoSequential(double EESetpoint, double EVSetpoint, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem){
        m_elevatorSubsystem = elevatorSubsystem;
        m_endEffectorSubsystem = endEffectorSubsystem;
        m_EESetpoint = EESetpoint;
        m_EVSetpoint = EVSetpoint;

        addRequirements(m_elevatorSubsystem, m_endEffectorSubsystem);
    }

    @Override
    public void initialize(){
        m_elevatorSubsystem.changeSetpoint(m_EVSetpoint, Constants.elevatorConstants.motorPowerLimit);
    }

    @Override
    public void execute(){
        if(m_elevatorSubsystem.atSetpoint()){
            m_endEffectorSubsystem.changeSetpoint(m_EESetpoint);
        }
    }

    @Override
    public boolean isFinished(){
        if(m_endEffectorSubsystem.atSetpoint() && m_elevatorSubsystem.atSetpoint()){
            return true;
        }
        else{
            return false;
        }
    }
}
