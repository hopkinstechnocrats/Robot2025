package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;

  public void Elevator() {
    elevatorMotor1 = new TalonFX(20, "Elevator Motor 1");
    elevatorMotor1.setVoltage(4);
    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor2 = new TalonFX(20, "Elevator Motor 2");
    elevatorMotor2.setVoltage(4);
    elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void elevatorUp() {
      elevatorMotor1.set(ElevatorConstants.elevatorSpeedUp);
      elevatorMotor2.set(ElevatorConstants.elevatorSpeedUp);
  }

  public void elevatorDown() {
    elevatorMotor1.set(ElevatorConstants.elevatorSpeedDown);
    elevatorMotor2.set(ElevatorConstants.elevatorSpeedDown);
  }

  public void stopElevator() {
    elevatorMotor1.set(0);
    elevatorMotor2.set(0);
  }
}