package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

public class ArmSubsystemController  {
    Wrist wrist;

    ArmSubsystemController(Wrist wrist){
        wrist = wrist;
    }

    public void liftArm(double angle){
        //shoulder.rotate(angle);
       double currentPitch = wrist.setPitch(180 - angle);
    }
}
