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
            driveForwards(swerveSubsystem).withTimeout(5)
        );
    }

    public Command pushLeftAuto(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            driveLeft(swerveSubsystem).withTimeout(5),
            driveForwards(swerveSubsystem).withTimeout(5)
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
                swerveSubsystem.drive(new Translation2d(0,1), 0, false);
            }, swerveSubsystem);
    }
    
}
