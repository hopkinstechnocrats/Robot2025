package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.ErrorCode;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommands {
    public static Command setLEDs(int r,
 int g,
 int b,
 int w,
 int startIdx,
 int count) {
return Commands.runOnce(() -> {
             LEDSubsystem.setLEDs();
 }, LEDSubsystem.ledsubsystem);
 }
}

