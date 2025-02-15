package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX rightMotor;
    private TalonFX leftMotor;

    public ElevatorSubsystem(){
        rightMotor = new TalonFX(11);
        leftMotor = new TalonFX(12);

        rightMotor.setVoltage(4);
        leftMotor.setVoltage(4);

        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void up(){
        rightMotor.set(0.5);
    }

    public void down(){
        rightMotor.set(-0.5);
    }

    public void brake(){
        rightMotor.set(0);
    }
}
