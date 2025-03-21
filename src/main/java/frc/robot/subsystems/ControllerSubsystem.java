package frc.robot.subsystems;
import java.util.function.Function;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

// private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driver_orientation);
//controller = driver_orientation

public class ControllerSubsystem extends SubsystemBase{
    private final double deadbandLimit = 0.5;
  private final NetworkTableInstance inst;
     private final NetworkTable table;
     private final NetworkTableEntry nt_drive_stick_section;

   public ControllerSubsystem(CommandXboxController controller){

      inst = NetworkTableInstance.getDefault();
      table = inst.getTable("Controller");
      nt_drive_stick_section = table.getEntry("Controls");
 

 
 
    }
 public void getTargetHeading(CommandXboxController controller){
   final double angle =((Math.atan2(controller.getRightX(),-controller.getRightY())) /Math.PI) * 180; // in degrees
   System.out.println("ff");
   if (angle <=90 && angle >=0)
      System.out.println("yay");
  if(
     Math.hypot(controller.getRightX(),-controller.getRightY()) <= deadbandLimit) {
    System.out.println("deadband area");
     } else {
   if(angle >= 30 && angle < 90)
      System.out.println("section 2");
   if(angle >= 90 && angle < 150)
      System.out.println("section 3");
   if(angle >= 150 && angle < 210)
      System.out.println("section 4");
   if(angle >= 210 && angle < 270)
      System.out.println("section 5");
   if(angle >= 270 && angle < 330)
      System.out.println("section 6");
   if(angle >=330 || angle < 30)
      System.out.println("section 1");
   

     }
  }



} 

