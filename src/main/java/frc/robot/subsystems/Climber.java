package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    TalonFX climbMotor;
    Servo climbingServo = new Servo(Constants.servoID);

    public Climber(){
        climbMotor = new TalonFX(16);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        climbMotor.setVoltage(4);
    }

    public void climb(){
        climbMotor.set(1);
    }

    public void brake(){
        climbMotor.stopMotor();
    }

    public void retractClimber(){
        climbingServo.setAngle(0);
    }

    public void extendClimber(){
        climbingServo.setAngle(90);
    }
}
