package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class Limelight {

    public static double llAngleWithVertical = 19;
    public static double llHeight = 9;
    public static double Kp = 0.02;
    public static double Kpx = 0.05;

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private LLResult result;
    private LLStatus status;

    private final int yellowSampleColorPipeline = 0;
    private final int redSampleColorPipeline = 1;
    private final int blueSampleColorPipeline = 2;
    private final int aprilTagPipeline = 3;


    /**
     * Creates a limelight
     *
     * @param hardwareMap hardwareMap
     * @param telemetry   telemetry
     */
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        configureLimelight();
    }

    /**
     * Configures the limelight
     */
    private void configureLimelight() {
        limelight.pipelineSwitch(yellowSampleColorPipeline);
        limelight.setPollRateHz(250);
        limelight.start();
    }

    /**
     * Gets the latest result from the set pipeline
     */
    private void getResult() {
        result = limelight.getLatestResult();
    }

    private void getStatus() {
        status = limelight.getStatus();
    }

    /**
     * Gets the Tx angle if the current pipeline is color
     *
     * @return the Tx angle
     */
    private double getTx() {
        getStatus();
        getResult();

        if (result != null && (status.getPipelineIndex() <= 2)) {
            return result.getTx();
        } else {
            return 0.0;
        }
    }

    /**
     * Gets the Ty angle if the current pipeline is color
     *
     * @return the Ty angle
     */
    private double getTy() {
        getStatus();
        getResult();

        if (result != null && (status.getPipelineIndex() <= 2)) {
            return result.getTy();
        } else {
            return 0.0;
        }
    }

    /**
     * Gets the x distance from the shoulder to the target
     * if the current pipeline is color.
     *
     * @return the distance to the target
     */
    public double getDistance() {
        getResult();
        getStatus();
        double xDistance;
        double[] Ty = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        double retryCount = 0;
        double filteredTy;
        final double MAX_RETRIES = 20;

        // how many degrees is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = llAngleWithVertical;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = llHeight;

        // distance from the target to the floor
        //1.1 for a sample
        double goalHeightInches = 1.1;


        if (result != null && (status.getPipelineIndex() <= 2)) {
            for (int i = 0; i < 16; ) {
                if (result != null) {
                    Ty[i] = getTy();
                    i++;
                } else {
                    // Retry limit reached; break out of the loop to prevent infinite loop
                    if (++retryCount >= MAX_RETRIES) {
                        telemetry.addLine("Failed to get result after 12 retries");
                        telemetry.update();
                        break;
                    }

                }
            }

            filteredTy = Arrays.stream(Ty).average().orElse(0.0);

            double angleToGoalDegrees = 90 - limelightMountAngleDegrees + filteredTy;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            xDistance = (limelightLensHeightInches - goalHeightInches) * Math.tan(angleToGoalRadians);
            return xDistance + 4.5;
        } else {
            return 0.0;
        }


    }

    /**
     * Returns the power for the motors to rotate
     * towards the target. Returns a non-zero power
     * until the angle Tx is close to zero.
     *
     * @return the power to assign to the motors
     */
    public double rotateBot() {
        getStatus();
        double Tx = getTx();
        double power;
        if (Tx > 0.8 || Tx < -0.8) {
            power = -Kpx * Tx;
        } else {
            power = 0.0;
           /* telemetry.addLine("Rotated Until Target");
            telemetry.update();*/
        }

        return power;

        /*telemetry.addData("Motor power:",power);
        telemetry.update();*/
    }

    public double positionBot() {
        double distance = getDistance() - 4.5;
        double error = distance - 15;
        if (error > 0.5) {
            return error * Kp + 0.2;
        } else if (error < 0.5) {
            return error * Kp - 0.2;
        } else {
            return 0.0;
        }
    }

    /**
     * Sets the pipeline to detect yellow samples.
     */
    public void detectYellow() {
        limelight.pipelineSwitch(yellowSampleColorPipeline);
    }

    /**
     * Sets the pipeline to detect red samples.
     */
    public void detectRed() {
        limelight.pipelineSwitch(redSampleColorPipeline);
    }

    /**
     * Sets the pipeline to detect blue samples.
     */
    public void detectBlue() {
        limelight.pipelineSwitch(blueSampleColorPipeline);
    }

    /**
     * Setts the pipeline to detect aprilTags.
     */
    public void detectAprilTags() {
        limelight.pipelineSwitch(aprilTagPipeline);
    }

}
