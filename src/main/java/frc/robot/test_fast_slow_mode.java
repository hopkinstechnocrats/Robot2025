/*
in robot container.  how do we drive?  what is default command?
    driveFieldOrientedAnglularVelocity

how is it made?
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

how is driveAngularVelocity passed?

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() *-1,
                                                                () -> driverXbox.getLeftX() *-1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

how can we make this work with slower speed?
    multiply getleftx and getlefty by some constant (.6?)

n

now rebuild from the beginning
    driveAngularVelocity_SLOW

    Command driveFieldOrientedAnglularVelocity_SLOW = drivebase.driveFieldOriented(driveAngularVelocity_SLOW);

we have infrastructre now.  binid it to a button

x.whiletrue(command.runonce(driveFieldOrientedAnglularVelocity_SLOW).repeatedly())


















*/