package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDSubsystem{ 
    public static int LEDOffset = 8;

    private CANdle candle;
} // I have no idea why this bracket is broken

public LEDSubsystem() {
    System.out.println("Initializing CANdle");

    CANdle candle = new CANdle(0);

     CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
} // This one is too

public static Command setLEDs(int r,
 int g,
 int b,
 int w,
 int startIdx,
 int count) {

 } // Seriously why?