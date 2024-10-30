package org.firstinspires.ftc.teamcode.models.worldModel;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class WorldObject {
    public String name;
    public String key;
    public Pose2d position;
    public RelativePosition relativePosition;
    public WorldObjectSize size;

    public WorldObject(String name, String key, Pose2d position, WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.position = position;
        this.size = size;
    }

    public WorldObject(String name, String key, RelativePosition relativePosition, WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.relativePosition = relativePosition;
        this.size = size;
    }

    @Override
    public String toString() {
        return "\nWorldObject [key = " + key + ", position =" + position + ", size  =" + size + "]";
    }
}
