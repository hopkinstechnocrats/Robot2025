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
            drive(swerveSubsystem, 1 ,0).withTimeout(1.5) 
            //3 seconds = a little over 9 feet
            //1.5 seconds = 4 and 1/3 feet
        );
    }

    public Command pushLeftAuto(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            //TODO: use gyroscope to drive in auto
            drive(swerveSubsystem, -1,0 ).withTimeout(1.5),
            drive(swerveSubsystem, 0, 0).withTimeout(.5),
            drive(swerveSubsystem, 0, .1).withTimeout(.5),
            drive(swerveSubsystem, 0, 1).withTimeout(2), //2 seconds = about 5.5 feet 
            drive(swerveSubsystem, 0, 0).withTimeout(.5),
            drive(swerveSubsystem, .1, 0).withTimeout(.5),
            drive(swerveSubsystem, 1, 0).withTimeout(4)
        );
    }

    public Command drive(SwerveSubsystem swerveSubsystem, double vx, double vy){
        return new RunCommand(
            () -> {
                swerveSubsystem.drive(new Translation2d(vx,vy), 0, false);
            }, swerveSubsystem);
    }
    
}
