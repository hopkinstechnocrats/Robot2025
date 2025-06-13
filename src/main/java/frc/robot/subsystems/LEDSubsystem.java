package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{ 
    public static int LEDOffset = 8;
    public static CANdle candle;
 

public LEDSubsystem() {
    System.out.println("Initializing CANdle");

    candle = new CANdle(20, "rio");

     CANdleConfiguration configAll = new CANdleConfiguration();
    //configAll.statusLedOffWhenActive = false;
    //configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    //configAll.brightnessScalar = 0.5;
    //configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
} 

public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count, Subsystem leds){
    candle.setLEDs(r, g, b, w, startIdx, count);
    return ErrorCode.OK;
}   
}
