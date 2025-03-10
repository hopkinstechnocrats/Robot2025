
//public class ControllerSubsystem extends SubsystemBase {}
// import xboxcontroller as controller
// deadband constant
//    private final NetworkTableInstance inst;
//    private final NetworkTable table;
//    private final NetworkTableEntry nt_drive_stick_section;

//public controllersunsystem(xboxcontroller controller){
// this.controller = controller
//inst = NetworkTableInstance.getDefault();
//table = inst.getTable("Controller");
// nt_drive_stick_section = table.getentry('joystick section')
//}

// private int joysticksection(x,y)
// angle = todegrees(atan2(x,y))
//deadband: if hypot(x,y) <= deadband return bad
// if angle 0<=angle<=60:  return section 1

//public double gettargetheading
// x = controller right x  //same y
// int section = joysticksection(x,y)
// return section
//nt_drive_stick_section.setDouble(section);

//stretch goals: apply normalization.  Make sure angle between 0 and 360,
//add if before 0, subtract if above 360
