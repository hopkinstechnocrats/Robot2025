package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeAlgefier extends SubsystemBase{
    VictorSP extender;
    VictorSP algaeMotor; 
    public DeAlgefier(){
        extender = new VictorSP(0);
        algaeMotor = new VictorSP(1);
    }

    public void extend(){
        extender.set(0.3);
    }

    public void retract(){
        extender.set(0);
    }

    public void dealgaefy(){
        algaeMotor.set(0.8);
    }

    public void brake(){
        algaeMotor.set(0);
    }
}
