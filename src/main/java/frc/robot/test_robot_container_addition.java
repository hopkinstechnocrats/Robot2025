import frc.robot.subsystems.ControllerSubsystem; 

private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driverXbox); 

controllerSubsystem.setDefaultCommand(
  new RunCommand(() -> {
    double targetHeading = controllerSubsystem.getTargetHeading();
  }, controllerSubsystem)
);
