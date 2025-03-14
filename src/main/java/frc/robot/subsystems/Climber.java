package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    TalonFX climbMotor;
    VictorSP victorsp = new VictorSP(2);

    public Climber(){
        climbMotor = new TalonFX(14);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        climbMotor.setVoltage(4);

        var slot0Configs = new TalonFXConfiguration().Slot0;
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        climbMotor.getConfigurator().apply(slot0Configs);

    }

    public void extend(){
        climbMotor.set(-0.4);
    }

    public void retract(){
        climbMotor.set(0.4);
    }

    public void brake(){
        climbMotor.stopMotor();
    }

    public void spinVictor(){
        victorsp.set(.5);
    }

    public void brakeVictor(){
        victorsp.set(0);
    }
}
