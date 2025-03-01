// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.concurrent.ConcurrentHashMap.KeySetView;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(8);

  //TODO: put actual servo id here
  public static final int servoID = 2;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class endEffectorConstants{
    public static final double kP = 0.2*8;
    public static final double kI = 0.025 / 2;
    public static final double kD = 0;
    public static final double motorPowerLimit = .1; //percent of max 1 (DO NOT SET TO NEAR 1)
    public static final double rotationsPerRevolution = 16*2*2 /*gear ratio, 16:1 gear box and 2 2:1 belts*/;
    public static final double LeftScore = 0.247; //position when scoring on left in rotations
    public static final double RightScore = -.215; //-0.193 - 1.9/360; //position when scoring on right in rotations
    public static final double LeftScoreL4 = .285;//0.27 + 2/360; //position when scoring on left in rotations
    public static final double RightScoreL4 = -.245;//-0.226 + 6/360; //position when scoring on right in rotations
    public static final double Stowage = 0.0; //stowage position for driving
    public static final int eeCANID = 13;
    public static final int throughboreCANID = 25;
    public static final double kEEAbsEncoderOffset = -0.07;
  }
  public final class elevatorConstants{
    public static final double kP = 0.2;
    public static final double kI = 0.025 / 10;
    public static final double kD = 0;
    public static final double minMotorHeight = 0.5; //rotations below which we stop down power (actually positive)
    public static final double motorPowerLimit = .2; //percent of max 1 (DO NOT SET TO NEAR 1)
    public static final double inchesPerRevolution = 1.751 * Math.PI;
    public static final double rotationsPerRevolution = 12/*gear ratio*/;
    public static final double rotationsPerInch = rotationsPerRevolution/inchesPerRevolution / 3 /*elevator stages (divided by 3 because its a cascade elevator)*/; //how many encoder ticks per inch of elevator movement
    public static final double L2Height = 7.625 + 13 + 2; //position when scoring on L2 inches
    public static final double L3Height = 13.25 + 23 + 2; //position when scoring on L3 inches
    public static final double L4Height = 21.75 + 38 + 6 + 0.8; //position when scoring on L4 inches
    public static final double L2HeightEnd = 11; //position when scoring on L2 inches
    public static final double kS = 0;
    public static final double kG = 0.22;
    public static final double kV = 10.66;
  }
}
