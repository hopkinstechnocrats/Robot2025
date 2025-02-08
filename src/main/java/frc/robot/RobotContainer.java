// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.limelight.driveAimAtTarget;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import org.dyn4j.geometry.Rotation;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final static CommandXboxController driverXboxRaw = new CommandXboxController(0);
    
    Trigger xButton = driverXboxRaw.x();
    Trigger yButton = driverXboxRaw.y();
    Trigger aButton = driverXboxRaw.a();
    Trigger bButton = driverXboxRaw.b();
    Trigger leftStick = driverXboxRaw.leftStick();
    Trigger rightStick = driverXboxRaw.rightStick();
    Trigger leftBumper = driverXboxRaw.leftBumper();
    Trigger rightBumper = driverXboxRaw.rightBumper();
    Trigger leftTrigger = driverXboxRaw.leftTrigger();
    Trigger righTrigger = driverXboxRaw.rightTrigger();

    

        // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        "swerve"));
    private final SendableChooser<Command> autoChooser;
    /**
    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> driverXboxRaw.getLeftY() * -1, () -> driverXboxRaw.getLeftX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
    * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
    */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(driverXboxRaw::getRightX,driverXboxRaw::getRightY)
        .headingWhile(true);

    /**
    * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
    */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
        .robotRelative(true).allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> -driverXboxRaw.getLeftY(), () -> -driverXboxRaw.getLeftX())
        .withControllerRotationAxis(() -> driverXboxRaw.getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(() ->
    Math.sin(driverXboxRaw.getRawAxis(2) * Math.PI) * (Math.PI *2),
        () ->
            Math.cos(driverXboxRaw.getRawAxis(2) * Math.PI) * (Math.PI *2))
        .headingWhile(true);

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        boolean isCompetition = false;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );
 
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
        * Use this method to define your trigger->command mappings. Triggers can be created via the
        * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
        * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
        * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
        * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
    */

    public static void setRightRumbleDriver(double rumble){
    driverXboxRaw.getHID().setRumble(RumbleType.kRightRumble, rumble);
    }

    private void configureBindings()
    {
        driverXboxRaw.y().whileTrue(new driveAimAtTarget(drivebase, ()->MathUtil.applyDeadband(-driverXboxRaw.getLeftY(), OperatorConstants.DEADBAND), ()->MathUtil.applyDeadband(-driverXboxRaw.getLeftX(), OperatorConstants.DEADBAND)));

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngleKeyboard);

        if(RobotBase.isSimulation()){
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        }
        else{
            drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
        }

        if(Robot.isSimulation()){
            driverXboxRaw.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXboxRaw.button(688398).whileTrue(drivebase.sysIdDriveMotorCommand());
        }
        if(DriverStation.isTest()){
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driverXboxRaw.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXboxRaw.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driverXboxRaw.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXboxRaw.back().whileTrue(drivebase.centerModulesCommand());
            driverXboxRaw.leftBumper().onTrue(Commands.none());
            driverXboxRaw.rightBumper().onTrue(Commands.none());
        }else{
            driverXbox.y().whileTrue(new driveAimAtTarget(drivebase, ()->MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND), ()->MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND)));
            driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
            //driverXbox.b().whileTrue(
            //    drivebase.driveToPose(
            //    new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
            driverXbox.start().whileTrue(Commands.none());
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());
        }
    }

    /**
        * Use this to pass the autonomous command to the main {@link Robot} class.
        *
        * @return the command to run in autonomous
    */
    public Command getAutonomousCommand(){
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
    }
}
