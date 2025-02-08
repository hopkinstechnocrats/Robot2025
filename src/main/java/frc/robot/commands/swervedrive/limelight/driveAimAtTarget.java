// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.limelight;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.limelightLib.LimelightHelpers;
import frc.robot.limelightLib.LimelightHelpers.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class driveAimAtTarget extends Command {
  private final SwerveSubsystem SwerveSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
  // Creates a new driveAimAtTarget.
  public driveAimAtTarget(SwerveSubsystem s_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SwerveSub.isValidVisionTarget()){
      System.out.println("April Tag Seen!" + LimelightHelpers.getFiducialID("limelight"));
      //heading = Math.pow(-SwerveSub.getVisionAngle()/30,3);
      heading = SwerveSub.getVisionAngle()/70;
      RobotContainer.setRightRumbleDriver(0);
    }else{
      //System.out.println("Warning: Swerve Aim: Lost Target!");
      RobotContainer.setRightRumbleDriver(1);
      heading = 0;
    }
    lastGoodHeading = heading;

    //System.out.println("Target is: " + heading);
    //SwerveSub.driveCommand(translationX, translationY, heading);


    SwerveSub.getSwerve().drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * SwerveSub.getSwerve().getMaximumChassisVelocity(),
    Math.pow(translationY.getAsDouble(), 3) * SwerveSub.getSwerve().getMaximumChassisVelocity()),
    heading * SwerveSub.getSwerve().getMaximumChassisAngularVelocity(),
    true,
    false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.setRightRumbleDriver(0);
    //System.out.println("Warning: Swerve Aim: Target Lost!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !SwerveSub.isValidVisionTarget();
    return false;
  }
}