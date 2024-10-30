package org.firstinspires.ftc.teamcode.ftc7083.models;

public class RelativePosition {
    public int referenceTagId;
    public double range;
    public double bearing;
    public double yaw;

    public RelativePosition(int referenceTagId,double range,double bearing,double yaw){
        this.referenceTagId=referenceTagId;
        this.range=range;
        this.bearing=bearing;
        this.yaw=yaw;
    }
}
