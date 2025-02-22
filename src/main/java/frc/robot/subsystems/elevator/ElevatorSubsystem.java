package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
    private Double m_setpoint = 0.0;
    private Double m_offset = 0.0;
    private final PIDController pidController;
    final ElevatorFeedforward ff = new ElevatorFeedforward(Constants.elevatorConstants.kS,
        Constants.elevatorConstants.kG,
        Constants.elevatorConstants.kV);
    NetworkTableInstance inst;
         NetworkTable table;
         NetworkTableEntry state;
         NetworkTableEntry desiredState;
    public ElevatorSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Elevator");
        state = table.getEntry("Current State");
        desiredState = table.getEntry("Desired State");
        final ElevatorFeedforward ff;

        rightMotor = new TalonFX(11);
        leftMotor = new TalonFX(12);

        rightMotor.setVoltage(4);
        leftMotor.setVoltage(4);

        //rightMotor.setPosition(0.0);
        //leftMotor.setPosition(0.0);

        m_offset = rightMotor.getPosition().getValueAsDouble();

        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
        pidController = new PIDController(Constants.elevatorConstants.kP,
            Constants.elevatorConstants.kI, Constants.elevatorConstants.kD);
        pidController.setTolerance(0.1);

        //rightMotor.setPosition(0.0,0.5);
        //leftMotor.setPosition(0.0,0.5);
     
    }
     public void moveToSetpoint(){
        pidController.setSetpoint(m_setpoint);
            final double measurement = rightMotor.getPosition().getValueAsDouble() - m_offset;
            final double command = MathUtil.clamp(
              ff.calculate(m_setpoint, 1) +
              pidController.calculate(measurement), -0.1, 0.1);
              rightMotor.set(command);
              state.setDouble(rightMotor.getPosition().getValueAsDouble());
              desiredState.setDouble(m_setpoint);
    }

    public void changeSetpoint(double setpoint){
      m_setpoint = -setpoint * elevatorConstants.rotationsPerInch; //setpoint is in rotations
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
    
    public void zeroMotors(){
    rightMotor.setPosition(0.0,0.5);
    leftMotor.setPosition(0.0,0.5);
    }
    
    /*//final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  public void bottom(){
    //rightMotor.setControl(m_request.withPosition(1.5));
    moveToSetpoint(0);
  }
  public void level2(){
    //rightMotor.setControl(/m_request.withPosition(2.5));
    moveToSetpoint(10);
  }
  public void level3(){
    //rightMotor.setControl(m_request.withPosition(3.5));
    moveToSetpoint(20);
  }
  public void level4(){
    //rightMotor.setControl(m_request.withPosition(4.5));
    moveToSetpoint(30); */
  }

