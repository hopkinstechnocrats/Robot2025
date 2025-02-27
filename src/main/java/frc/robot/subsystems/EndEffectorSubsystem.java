package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.endEffectorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase{
    private TalonFX motor;
    private Double m_setpoint = 0.0; // Rotations pre-gearbox
    private Double m_offset = 0.0; //Also rotations
    private Long m_counter = 0L;
    private final PIDController pidController;
    private CANcoder throughbore;
    NetworkTableInstance inst;
         NetworkTable table;
         NetworkTableEntry nt_measurement;
         NetworkTableEntry nt_command;
         NetworkTableEntry nt_offset;
         NetworkTableEntry nt_setpoint;
         NetworkTableEntry nt_changed;
         NetworkTableEntry nt_object_a;
         NetworkTableEntry nt_object_b;
         NetworkTableEntry nt_absolute;
         NetworkTableEntry nt_motor_position;
    public EndEffectorSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("EndEffector");
        nt_measurement = table.getEntry("Measurement [rot]");
        nt_command = table.getEntry("Command [-1 to 1]");
        nt_offset = table.getEntry("Offset [rot]");
        nt_setpoint = table.getEntry("Setpoint [rot]");
        nt_changed = table.getEntry("Setpoint set [rot]");
        nt_object_a = table.getEntry("Points North");
        nt_object_b = table.getEntry("Points South");
        nt_absolute = table.getEntry("Absolute Encoder Position [rot]");
        nt_motor_position = table.getEntry("Position of Motor [rot]");

        motor = new TalonFX(endEffectorConstants.eeCANID);
        motor.setVoltage(4);

        throughbore = new CANcoder(21);

        motor.setNeutralMode(NeutralModeValue.Brake);
      

        pidController = new PIDController(Constants.endEffectorConstants.kP,
            Constants.endEffectorConstants.kI, Constants.endEffectorConstants.kD);
        pidController.setTolerance(0.1);

     
    }
     public void moveToSetpoint(){
        pidController.setSetpoint(m_setpoint);
        final double measurement = throughbore.getPosition().getValueAsDouble();
        double command = MathUtil.clamp(
        
         pidController.calculate(measurement), -endEffectorConstants.motorPowerLimit, endEffectorConstants.motorPowerLimit);  
         m_counter++;
        motor.set(0/*command*/);
        nt_measurement.setDouble(measurement);
        nt_setpoint.setDouble(m_setpoint);
        nt_offset.setDouble(m_offset);
        nt_command.setDouble(command);
        nt_object_a.setInteger(m_counter);
        nt_absolute.setDouble(throughbore.getAbsolutePosition().getValueAsDouble());
        nt_motor_position.setDouble(motor.getPosition().getValueAsDouble()
                / Constants.endEffectorConstants.rotationsPerRevolution);
    }

    public void changeSetpoint(double setpoint){
      
      m_setpoint = setpoint; //setpoint is in rotations
      nt_changed.setDouble(m_setpoint);
      nt_object_b.setInteger(m_counter);
      
    }

  
  }

