package frc.robot.subsystems;
import java.util.function.Function;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driver_orientation);
//controller = driver_orientation

public class ControllerSubsystem{
    private final double deadbandLimit = 0.5;
  private final NetworkTableInstance inst;
     private final NetworkTable table;
     private final NetworkTableEntry nt_drive_stick_section;

    public ControllerSubsystem(CommandXboxController controller){

 inst = NetworkTableInstance.getDefault();
 table = inst.getTable("Controller");
 nt_drive_stick_section = table.getEntry("Controls");
 

 
 
 double angle =((Math.atan2(controller.getRightX(),-controller.getRightY())) /Math.PI) * 180;
 if(
    Math.hypot(controller.getRightX(),-controller.getRightY()) <= deadbandLimit)
   System.out.println("deadband area");

if(angle <= 30 && angle <= 330)
System.out.println("Section 1");
if(angle >= 270 && angle <= 329)
System.out.println("Section 2");
if(angle >= 210 && angle <= 269)
System.out.println("Section 3");
if(angle >= 150 && angle <= 209)
System.out.println("Section 4");
if(angle >= 90 && angle <= 149)
System.out.println("Section 5");
if(angle >= 31 && angle <=89)
System.out.println("Section 6");


//double gettargetheading;
// x = controller right x  //same y
// int section = joysticksection(x,y)
// return section
//nt_drive_stick_section.setDouble(section);

 }
 
}
