package frc.robot.subsystems;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerSubsystem extends SubsystemBase {
    private final double deadbandLimit = 0.5;
  private final NetworkTableInstance inst;
     private final NetworkTable table;
     //private final NetworkTableEntry nt_drive_stick_section;

    public ControllerSubsystem(CommandXboxController controller){
    final CommandXboxController driver_orientation = new CommandXboxController(2); 
 inst = NetworkTableInstance.getDefault();
 table = inst.getTable("Controller");
// nt_drive_stick_section = table.getentry('Controls');
 
 double angle =((Math.atan2(driver_orientation.getRightX(),-driver_orientation.getRightY())) /Math.PI) * 180;
 if(
    Math.hypot(driver_orientation.getRightX(),-driver_orientation.getRightY()) <= deadbandLimit)
  System.out.println("deadband area");
 //if angle 0<=angle<=60:  return section 1;




 }
}
