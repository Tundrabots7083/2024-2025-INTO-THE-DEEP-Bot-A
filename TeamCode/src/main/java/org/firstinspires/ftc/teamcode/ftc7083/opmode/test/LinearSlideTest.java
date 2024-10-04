
package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

@Config
@TeleOp(name = "Linear Slide Test", group = "tests")
public class LinearSlideTest extends OpMode {
    public static double LINEAR_SLIDE_LENGTH = 0.0;
    private LinearSlide linearSlide;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlide = new LinearSlide(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        linearSlide.setLength(LINEAR_SLIDE_LENGTH);
        linearSlide.execute();
        telemetry.addData("Target Length", linearSlide.getTargetLength());
        telemetry.addData("Current Length", linearSlide.getCurrentLength());
        telemetry.update();
    }

}
