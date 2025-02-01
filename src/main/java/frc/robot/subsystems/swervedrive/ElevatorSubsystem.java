package frc.robot.subsystems.swervedrive;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix6.configs;

public class elevator extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;

  public Climb() {
    elevatorMotor1 = new TalonFX(20, "Elevator Motor 1");
    elevatorMotor1.setVoltage(4);
    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor2 = new TalonFX(20, "Elevator Motor 2");
    elevatorMotor2.setVoltage(4);
    elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void elevatorUp() {
    elevatorMotor1.set(ClimbConstants.climbSpeedUp);
    elevatorMotor2.set(ClimbConstants.climbSpeedUp);
  }

  public void climbDown() {
    elevatorMotor1.set(ClimbConstants.climbSpeedDown);
    elevatorMotor2.set(ClimbConstants.climbSpeedDown);
  }

  public void noClimb() {
    elevatorMotor1.set(0);
    elevatorMotor2.set(0);
  }
}