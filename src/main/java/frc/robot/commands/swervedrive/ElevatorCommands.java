package frc.robot.commands.swervedrive;

import frc.robot.subsystems.swervedrive.ElevatorSubsystem;

import javax.management.OperationsException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.ElevatorCommands;

import java.beans.Encoder;
import java.io.File;
import java.time.Year;

import swervelib.SwerveInputStream;

public class ElevatorCommands {
  public final double encoderRotation = 0;
    private ElevatorCommands() {}
    public final CommandXboxController operatorControllerElevator = new CommandXboxController(1);
    public Encoder encoder1 = new Encoder();
      public double encoder1rotation = encoder1.get();
    }

    public static Command elevatorUp(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorUp();
            },
            elevator);
      }
    
      public static Command elevatorDown(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.elevatorDown();
            },
            elevator);
      }

      public static Command stop(ElevatorSubsystem elevator) {
        return Commands.run(
            () -> {
              elevator.stopElevator();
            },
            elevator);
      }
      public Command level1(ElevatorSubsystem elevator) {
        return Commands.run(
          () ->   {
          
            if (encoderRotation < 1.5) 
              elevator.elevatorUp();
            }
            ,

            
            elevator);
          }

        }
      
        
      
      
        
      
    
      