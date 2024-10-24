package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

import java.util.List;


@TeleOp(name = "Sensor: Limelight3A", group = "test")
@Config
public class SensorLimelight3A extends LinearOpMode {

    private Limelight3A limelight;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;
    private double filteredDistance = 0.0;

    public static double llHeight = 9.2;
    public static double llAngleWithVertical = 15;
    public static double Ks = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        leftFront = new Motor(hardwareMap,telemetry,"leftFront");
        rightFront = new Motor(hardwareMap,telemetry,"rightFront");
        leftBack = new Motor(hardwareMap,telemetry,"leftRear");
        rightBack = new Motor(hardwareMap,telemetry,"rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());


                  /*  // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        }*/


                    telemetry.addData("Distance to target:",calculateDistance(result));
                    telemetry.update();

                    rotateBot(result);

                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }

    private double calculateDistance(LLResult result)  {

        double lastDistance = filteredDistance;

        double targetOffsetAngle_Vertical = result.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = llAngleWithVertical;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = llHeight;

        // distance from the target to the floor
        double goalHeightInches = 1.1;

        double angleToGoalDegrees = 90 - limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double rawDistance = (limelightLensHeightInches - goalHeightInches) * Math.tan(angleToGoalRadians);

        filteredDistance += (rawDistance - lastDistance) / Ks;

        return filteredDistance;

    }

    private void rotateBot(LLResult result) {
        double Tx = result.getTx();
        double power = 0.0;
        if(Tx > 0.5) {
            power = 0.1;
        } else if (Tx < -0.5) {
            power = -0.1;
        } else {
            power = 0.0;
            telemetry.addLine("Rotated Until Target");
            telemetry.update();
        }

        telemetry.addData("Motor power:",power);
        telemetry.update();

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
}
