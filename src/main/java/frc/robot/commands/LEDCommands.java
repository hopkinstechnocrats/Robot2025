package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommands {
    public static Command setLEDs(int r,
 int g,
 int b,
 int w,
 int startIdx,
 int count,
 LEDSubsystem leds) {
return Commands.runOnce(() -> {
             leds.setLEDs(r, g, b, w, startIdx, count, leds);
 }, leds);
 }
}

