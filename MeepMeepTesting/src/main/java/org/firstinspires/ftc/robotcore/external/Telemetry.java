package org.firstinspires.ftc.robotcore.external;

public class Telemetry {

    public void addData(String caption, Object value) {
        System.out.println(caption + ": " + value);
    }

    public void addLine(String lineCaption) {
        System.out.println(lineCaption);
    }
}
