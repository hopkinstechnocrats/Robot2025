package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
    }

    public void extend(){
        climbMotor.set(1);
    }

    public void retract(){
        climbMotor.set(-1);
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
