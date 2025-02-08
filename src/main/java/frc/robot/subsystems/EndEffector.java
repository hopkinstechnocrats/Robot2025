package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class  EndEffector extends SubsystemBase{
    TalonFX eeMotor;
    Canandmag throughBore;

    private PIDController pid;

    public EndEffector(){
        eeMotor = new TalonFX(20);
        eeMotor.setNeutralMode(NeutralModeValue.Brake);
        eeMotor.setVoltage(1);

        throughBore = new Canandmag(21);
        throughBore.setPosition(throughBore.getAbsPosition() 
            - Constants.endEffectorConstants.kEEAbsEncoderOffset);

        pid = new PIDController(Constants.endEffectorConstants.kp,
            Constants.endEffectorConstants.ki, Constants.endEffectorConstants.kd);
    }

    public void moveToSetpoint(double setpoint){
        eeMotor.set(pid.calculate(throughBore.getPosition(), setpoint));
    }
} 
