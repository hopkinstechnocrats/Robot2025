package frc.robot.autos;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

public class Autos {

    //move straight forward
    public Command forwardAuto(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            driveForwards(swerveSubsystem).withTimeout(1.5) 
            //3 seconds = a little over 9 feet
            //1.5 seconds = 4 and 1/3 feet
        );
    }

    public Command pushLeftAuto(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            driveLeft(swerveSubsystem).withTimeout(1),
            driveForwards(swerveSubsystem).withTimeout(1)
        );
    }

    public Command driveForwards(SwerveSubsystem swerveSubsystem){
        return new RunCommand(
            () -> {
                swerveSubsystem.drive(new Translation2d(1,0), 0, false);
            }, swerveSubsystem);
    }

    public Command driveLeft(SwerveSubsystem swerveSubsystem){
        return new RunCommand(
            () -> {
                swerveSubsystem.drive(new Translation2d(0,.5), 0, false);
            }, swerveSubsystem);
    }
    
}
