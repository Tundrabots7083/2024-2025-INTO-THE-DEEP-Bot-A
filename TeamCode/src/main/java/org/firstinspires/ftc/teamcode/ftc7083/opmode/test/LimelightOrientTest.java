package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;


@TeleOp(name = "Limelight Orient Test", group = "test")
    public class LimelightOrientTest  extends OpMode {

        private Limelight limelight;
        private MecanumDrive mecanumDrive;

        @Override
        public void init() {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            limelight = new Limelight(hardwareMap,telemetry);
            mecanumDrive = new MecanumDrive(hardwareMap,telemetry);

            telemetry.setMsTransmissionInterval(11);
            telemetry.addLine("Initialization Complete");
            telemetry.update();
        }

        @Override
        public void loop() {

            double xDistance = limelight.getDistance() - 4.5;
            double turnPower = limelight.rotateBot();
            double drivePower = limelight.positionBot();

            mecanumDrive.drive(0,drivePower,turnPower);

            telemetry.addData("Distance to Target:",xDistance);
            telemetry.addData("Turn Power:",turnPower);
            telemetry.addData("Drive Power:",drivePower);
            telemetry.update();
        }


    }