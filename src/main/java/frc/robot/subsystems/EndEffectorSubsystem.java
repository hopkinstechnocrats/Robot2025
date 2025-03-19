package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.endEffectorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase{
    private TalonFX motor;
    //private final PIDController pidController;
    // class member variable
    final PositionVoltage m_position = new PositionVoltage(0);
    // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
    final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(8, 16)
    );
  
    // Final target of 200 rot, 0 rps
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
//    private final ArmFeedforward feedforwards;
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
         NetworkTableEntry nt_feed_forwards;
         NetworkTableEntry nt_PID;
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
        nt_feed_forwards = table.getEntry("Feed Forwards command [-1 to 1]");
        nt_PID = table.getEntry("PID command [-1 to 1]");

        motor = new TalonFX(endEffectorConstants.eeCANID);
        motor.setVoltage(4);

        throughbore = new CANcoder(21);

        motor.setNeutralMode(NeutralModeValue.Brake);




        var slot0Configs = new TalonFXConfiguration().Slot0;
        slot0Configs.kP = 0; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        motor.getConfigurator().apply(slot0Configs);


//        feedforwards = new ArmFeedforward(0.05, 0.19, 1.26); 
    }
     public void moveToSetpoint(){
        // periodic, update the profile setpoint for 20 ms loop time
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
        // apply the setpoint to the control request
        m_position.Position = m_setpoint.position;
        m_position.Velocity = m_setpoint.velocity;
        motor.setControl(m_position);
        
        /**
        nt_measurement.setDouble(measurement);
        nt_setpoint.setDouble(m_setpoint);
        nt_offset.setDouble(m_offset);
        nt_command.setDouble(command);
        nt_object_a.setInteger(m_counter);
        nt_absolute.setDouble(throughbore.getAbsolutePosition().getValueAsDouble());
        nt_motor_position.setDouble(motor.getPosition().getValueAsDouble()
                / Constants.endEffectorConstants.rotationsPerRevolution);
        nt_PID.setDouble(PIDcommand);
        nt_feed_forwards.setDouble(FFcommand);
        */
    }

    

    public void changeSetpoint(double setpoint){
      m_goal = new TrapezoidProfile.State(setpoint, .1);
      
      //m_setpoint = setpoint; //setpoint is in rotations
      //nt_changed.setDouble(m_setpoint);
      //nt_object_b.setInteger(m_counter);
      
    }

  
  }

