package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX rightMotor;
    private TalonFX leftMotor;
    private final PIDController pidController;
    NetworkTableInstance inst;
         NetworkTable table;
         NetworkTableEntry state;
         NetworkTableEntry desiredState;
    public ElevatorSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Elevator");
        state = table.getEntry("Current State");
        desiredState = table.getEntry("Desired State");

        rightMotor = new TalonFX(11);
        leftMotor = new TalonFX(12);

        rightMotor.setVoltage(4);
        leftMotor.setVoltage(4);

        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
        pidController = new PIDController(Constants.elevatorConstants.kP,
            Constants.elevatorConstants.kI, Constants.elevatorConstants.kD);
        pidController.setTolerance(0.1);
         
    }
     public void moveToSetpoint(double setpoint){
        state.setDouble(rightMotor.getPosition().getValueAsDouble());
        desiredState.setDouble(setpoint);
        while(!pidController.atSetpoint()){
            rightMotor.set(MathUtil.clamp(
                pidController.calculate(setpoint), -1, 1));}
        
        }
    
    public void up(){
        rightMotor.set(-0.05);
    }

    public void down(){
        rightMotor.set(0.05);
    }

    public void brake(){
        rightMotor.stopMotor();
    }
    
    
    //final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  public void level1(){
    //rightMotor.setControl(m_request.withPosition(1.5));
  }
  public void level2(){
    //rightMotor.setControl(/m_request.withPosition(2.5));
  }
  public void level3(){
    //rightMotor.setControl(m_request.withPosition(3.5));
  }
  public void level4(){
    //rightMotor.setControl(m_request.withPosition(4.5));
  }
}
