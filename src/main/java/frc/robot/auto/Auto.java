package frc.robot.auto;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

public class Auto {

    public Command runAuto(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            driveForwards(swerveSubsystem).withTimeout(5)
        );
    }

    public Command driveForwards(SwerveSubsystem swerveSubsystem){
        return new RunCommand(
            () -> {
                swerveSubsystem.drive(new Translation2d(1,0), 0, false);
            }, swerveSubsystem);
    }
    
}
