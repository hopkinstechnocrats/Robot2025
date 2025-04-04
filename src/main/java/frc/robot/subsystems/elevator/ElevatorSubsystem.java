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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX rightMotor;
    private TalonFX leftMotor;
    private Double m_setpoint = 0.0; // Rotations pre-gearbox
    private Double m_offset = 0.0; //Also rotations
    private Double m_measurement = 0.0; // rotations
    private Long m_counter = 0L;
    private double m_speed = elevatorConstants.motorPowerLimit;
    private final PIDController pidController;
    final ElevatorFeedforward ff = new ElevatorFeedforward(Constants.elevatorConstants.kS,
        Constants.elevatorConstants.kG,
        Constants.elevatorConstants.kV);
    NetworkTableInstance inst;
         NetworkTable table;
         NetworkTableEntry nt_measurement;
         NetworkTableEntry nt_command;
         NetworkTableEntry nt_offset;
         NetworkTableEntry nt_setpoint;
         NetworkTableEntry nt_changed;
         NetworkTableEntry nt_object_a;
         NetworkTableEntry nt_object_b;
    public ElevatorSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Elevator");
        nt_measurement = table.getEntry("Measurement [rot]");
        nt_command = table.getEntry("Command [-1 to 1]");
        nt_offset = table.getEntry("Offset [rot]");
        nt_setpoint = table.getEntry("Setpoint [rot]");
        nt_changed = table.getEntry("Setpoint set [rot]");
        nt_object_a = table.getEntry("Points North");
        nt_object_b = table.getEntry("Points South");
        SmartDashboard.putNumber("kP", Constants.elevatorConstants.kP);
        SmartDashboard.putNumber("kD", Constants.elevatorConstants.kD);
        SmartDashboard.putNumber("kI", Constants.elevatorConstants.kI);
        final ElevatorFeedforward ff;

        rightMotor = new TalonFX(11);
        leftMotor = new TalonFX(12);
        //rightMotor.setPosition(0.0);
        //leftMotor.setPosition(0.0);

        m_offset = rightMotor.getPosition().getValueAsDouble();

        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
        pidController = new PIDController(Constants.elevatorConstants.kP,
            Constants.elevatorConstants.kI, Constants.elevatorConstants.kD);
        pidController.setTolerance(0.5);

        //rightMotor.setPosition(0.0,0.5);
        //leftMotor.setPosition(0.0,0.5);
     
    }

    @Override
    public void periodic() {
        moveToSetpoint();
    }

     public void moveToSetpoint(){
        pidController.setSetpoint(m_setpoint);
        m_measurement = rightMotor.getPosition().getValueAsDouble() - m_offset;
        double command = MathUtil.clamp(
         /*  -ff.calculate(m_setpoint, 1) +*/ /*negative is up, positive is down */ 
          pidController.calculate(m_measurement) - 0.033, -m_speed, m_speed);  
        if(m_measurement > -elevatorConstants.minMotorHeight)
        {
          command = MathUtil.clamp(command, -m_speed, 0.0);
        }
         m_counter++;
        rightMotor.set(command);
        nt_measurement.setDouble(m_measurement);
        nt_setpoint.setDouble(m_setpoint);
        nt_offset.setDouble(m_offset);
        nt_command.setDouble(command);
        nt_object_a.setInteger(m_counter);
    }

    public void changeSetpoint(double setpoint, double speed){
      
      m_setpoint = -setpoint * elevatorConstants.rotationsPerInch; //setpoint is in rotations
      m_speed = speed;
      nt_changed.setDouble(m_setpoint);
      nt_object_b.setInteger(m_counter);
    
      // System.out.println("EL -- Change setpoint to " + m_setpoint);
      
    }

    public boolean atSetpoint(){
      //System.out.println("Setpoint = " + m_setpoint + "  Actual = " + m_measurement);
      //return pidController.atSetpoint();
      return (Math.abs(m_measurement - m_setpoint) < 1);
    }

    public double getSetpoint(){
      //return pidController.atSetpoint();
      return (m_setpoint);
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

