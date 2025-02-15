package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class  EndEffector extends SubsystemBase{
    TalonFX eeMotor;

    public EndEffector(){
        eeMotor = new TalonFX(13);
        eeMotor.setNeutralMode(NeutralModeValue.Brake);
        eeMotor.setVoltage(1);
    }

    public void moveLeft(){
        eeMotor.set(0.1);
    }

    public void moveRight(){
        eeMotor.set(-0.1);
    }

    public void brake(){
        eeMotor.stopMotor();
    }
} 
