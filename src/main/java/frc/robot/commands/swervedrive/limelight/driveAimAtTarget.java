// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.limelight;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.limelightLib.LimelightHelpers;
import frc.robot.limelightLib.LimelightHelpers.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem.*;
import swervelib.math.SwerveMath;
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
    
    // April tag distances from the ground in inches
    double reefAprilDis = 6.875; // CORAL REEF
    double procAprilDis = 45.875; // PROCESSOR
    double statAprilDis = 53.25; // CORAL STATION
    double bargAprilDis = 69; // BARGE

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // Limelight angle from horizontal
    double limelightMountAngleDegrees = 0.0;
    // Center of limelight lens to the floor
    double limelightLensHeightInches = 16.0;
    // Distance from target to floor
    //double goalHeightInches = reefAprilDis;     // CHANGE FOR MULTIPLE TARGETS
    double goalHeightInches = 12.1; // TEST DISTANCE GET RID OF WHEN DONE

    //calculate distance

    
    double distanceFromLimelightToGoalInches = ((goalHeightInches - limelightLensHeightInches));
    double distanceMeters = distanceFromLimelightToGoalInches * 0.0254;
    double targetDistance = distanceMeters - 1; // Desired distance from april tag -- change the 1 value if needed
    //SwerveSub.driveToDistanceCommand(targetDistance, 0.1); // old drive code
    System.out.println("Distance to limelight: " + distanceMeters);
    System.out.println("Distance from limelight to goal: " + distanceFromLimelightToGoalInches);
    System.out.println("Distance to drive: " + targetDistance);

    if (distanceMeters != targetDistance && LimelightHelpers.getFiducialID("limelight") == 2) {
      if (distanceMeters > targetDistance) {
        //SwerveSub.drive(SwerveMath.scaleTranslation(new Translation2d(0.1, 0.1), 1), 0, false);
        SwerveSub.getSwerve().drive(new Translation2d(0.1 * SwerveSub.getSwerve().getMaximumChassisVelocity(),
          0 * SwerveSub.getSwerve().getMaximumChassisVelocity()),
          heading * SwerveSub.getSwerve().getMaximumChassisAngularVelocity(),
          false,
          false);
      } else {
        //SwerveSub.drive(SwerveMath.scaleTranslation(new Translation2d(-0.1, -0.1), 1), 0, false);
        SwerveSub.getSwerve().drive(new Translation2d(0.1 * SwerveSub.getSwerve().getMaximumChassisVelocity(),
          0 * SwerveSub.getSwerve().getMaximumChassisVelocity()),
          heading * SwerveSub.getSwerve().getMaximumChassisAngularVelocity(),
          false,
          false);
      }
    }

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