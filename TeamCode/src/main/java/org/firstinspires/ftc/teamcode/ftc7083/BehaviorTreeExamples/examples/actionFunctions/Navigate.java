package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.models.PIDNCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDriveSingleton;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;
@Config
public class Navigate implements ActionFunction {
        private NavigationType navigationType = NavigationType.RELATIVE;
        private DriveTrain driveTrain;

        private AprilTagDetection referenceTag = null;
        private AprilTagPoseFtc initialPose = null;
        private double drivePower = 0; // Desired forward power/speed (-1 to +1)
        private double strafePower = 0; // Desired strafe power/speed (-1 to +1)
        private double turnPower = 0; // Desired turning power/speed (-1 to +1)

        private DriveTrainConfig driveTrainConfig = null;
        private int referenceTagId = -1;
        private double headingErrorTolerance = 1;
        private double rangeErrorTolerance = 1;
        private double yawErrorTolerance = 1;
        private LinearOpMode opMode;
        private RelativePosition currentRelativePositionTarget;
        private RelativePosition currentAbsolutePositionTarget;
        private List<AprilTagDetection> currentDetections;

        private PIDController rangeController;
        private PIDFController rangeFController;
        private PIDCoefficients pidCoefficients;
        private PIDController headingController;
        private PIDFController headingFController;

        private PIDController yawController;
        private PIDFController yawFController;

        private PIDNCoeficients pidNCoeficients;


        private MotionProfile rangeMotionProfile;
        private MotionProfile headingMotionProfile;
        private MotionProfile yawMotionProfile;
        public static double RKp=0.033;
        public static double RKi=0.185;
        public static double RKd=0.005;//0.0135 use if tune

        public static double HKp=0.028;
        public static double HKi=0.024;
        public static double HKd=0.0001;

        public static double YKp=0.025;
        public static double YKi=0.09;
        public static double YKd =0.0001;

        ElapsedTime timer;
        private NanoClock clock;
        double currentStartTime;

    CenterStageORMechanumDriveSingleton drive = null;
    //SampleMecanumDrive drive1 = null;
        public Navigate(LinearOpMode opMode) {
                this.opMode = opMode;
                this.driveTrain = new DriveTrain(opMode);


                long MILLIS_IN_NANO = 1000000;
                timer = new ElapsedTime(MILLIS_IN_NANO);
                clock = NanoClock.system();
                currentStartTime = clock.seconds();

            GlobalStoreSingleton globalStore = GlobalStoreSingleton.getInstance(opMode);

            if(this.drive == null) {
                this.drive = CenterStageORMechanumDriveSingleton.getInstance(opMode.hardwareMap, opMode, globalStore);
            }

        }

        public Status perform(GlobalStoreSingleton globalStore) {
                Status status = Status.RUNNING;

            this.driveTrainConfig = (DriveTrainConfig) globalStore.getValue("DriveTrainConfig");

            this.navigationType = (NavigationType) globalStore.getValue("NavigationType");

                if(this.navigationType == NavigationType.RELATIVE){
                    status = navigateByAprilTags(globalStore);
                }

                if(this.navigationType == NavigationType.ABSOLUTE){
                        status = navigateByTrajectory(globalStore);
                }

               return status;
        }

        private Status navigateByAprilTags(GlobalStoreSingleton globalStore){
                this.driveTrainConfig = (DriveTrainConfig) globalStore.getValue("DriveTrainConfig");

                getCurrentRelativePositionTarget(globalStore);
                getErrorTolerances(globalStore);
                getCurrentDetections(globalStore);

                if (this.referenceTag == null || this.referenceTag.ftcPose == null) {
                        return Status.FAILURE;
                }
                setPIDControllers(globalStore);


                return navigateToRelativeLocation();

               // return navigateToRelativeLocationWithMotionProfile();
        }
        private void getCurrentRelativePositionTarget(GlobalStoreSingleton globalStore) {
                this.currentRelativePositionTarget = (RelativePosition) globalStore.getValue("CurrentTarget");
                this.referenceTagId = currentRelativePositionTarget.referenceTagId;
        }

        private void getErrorTolerances(GlobalStoreSingleton globalStore){
                ErrorTolerances errorTolerances =(ErrorTolerances) globalStore.getValue("ErrorTolerances");
                this.headingErrorTolerance=errorTolerances.headingErrorTolerance;
                this.rangeErrorTolerance = errorTolerances.rangeErrorTolerance;
                this.yawErrorTolerance = errorTolerances.yawErrorTolerance;
        }


        private void getCurrentDetections(GlobalStoreSingleton globalStore) {
                this.currentDetections = (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");

                if(this.currentDetections == null){
                    return;
                }
                this.referenceTag = currentDetections.stream().filter(detection -> detection.metadata != null && detection.id == this.referenceTagId).findFirst().orElse(null);

                if (referenceTag != null) {
                        if(initialPose == null){
                                initialPose=referenceTag.ftcPose;
                        }
                        opMode.telemetry.addData("Navigate", "Tag ID %d metadata.id %d name is %s\n", referenceTag.id, referenceTag.metadata.id, referenceTag.metadata.name);
                } else {
                        opMode.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", -1);
                }

                opMode.telemetry.update();
        }

        private void setPIDControllers(GlobalStoreSingleton globalStore){
                this.pidNCoeficients = (PIDNCoeficients) globalStore.getValue("PIDCoeficients");

                this.rangeController = new PIDController(this.pidNCoeficients.RKp, this.pidNCoeficients.RKi, this.pidNCoeficients.RKd,this.opMode, "R");


                //Set RoadRunner controller
                PIDCoefficients rangePIDCoeficients = new PIDCoefficients(this.pidNCoeficients.RKp,this.pidNCoeficients.RKi, this.pidNCoeficients.RKd);
                this.rangeFController = new PIDFController(rangePIDCoeficients);



                this.headingController = new PIDController(this.pidNCoeficients.HKp,this.pidNCoeficients.HKi,this.pidNCoeficients.HKd,this.opMode, "H");
                //Set RoadRunner controller
                PIDCoefficients headingPIDCoeficients = new PIDCoefficients(this.pidNCoeficients.HKp,this.pidNCoeficients.HKi,this.pidNCoeficients.HKd);
                this.headingFController = new PIDFController(headingPIDCoeficients);

                this.yawController = new PIDController(this.pidNCoeficients.YKp,this.pidNCoeficients.YKi,this.pidNCoeficients.YKd,this.opMode, "Y");
                //Set RoadRunner controller
                PIDCoefficients yawPIDCoeficients = new PIDCoefficients(this.pidNCoeficients.YKp,this.pidNCoeficients.YKi,this.pidNCoeficients.YKd);
                this.yawFController = new PIDFController(yawPIDCoeficients);

               /* if(this.rangeController == null) {
                        this.rangeController = new PIDController(this.pidCoeficients.RKp, this.pidCoeficients.RKi, this.pidCoeficients.RKd, this.opMode, "R");
                }
                if(this.headingController == null) {
                        this.headingController = new PIDController(this.pidCoeficients.HKp, this.pidCoeficients.HKi, this.pidCoeficients.HKd, this.opMode, "H");
                }

                if(this.yawController == null) {
                        this.yawController = new PIDController(this.pidCoeficients.YKp, this.pidCoeficients.YKi, this.pidCoeficients.YKd, this.opMode, "Y");
                }


                if(this.rangeController == null) {
                        this.rangeController = new PIDController(this.RKp, this.RKi, this.RKd, this.opMode, "R");
                }
                if(this.headingController == null) {
                        this.headingController = new PIDController(this.HKp, this.HKi, this.HKd, this.opMode, "H");
                }

                if(this.yawController == null) {
                        this.yawController = new PIDController(this.YKp, this.YKi, this.YKd, this.opMode, "Y");
                }*/

        }

        private Status navigateToRelativeLocation() {
                if (this.referenceTag == null || this.referenceTag.ftcPose == null) {
                        return Status.FAILURE;
                }

                double rangeError = referenceTag.ftcPose.range - this.currentRelativePositionTarget.range;
                double headingError = referenceTag.ftcPose.bearing - this.currentRelativePositionTarget.bearing;
                double yawError = referenceTag.ftcPose.yaw - this.currentRelativePositionTarget.yaw;

                opMode.telemetry.addData("Navigate1", "SetPoint range: %f heading: %f yaw: %f\n", currentRelativePositionTarget.range, currentRelativePositionTarget.bearing, currentRelativePositionTarget.yaw);
                opMode.telemetry.addData("Navigate1", "navigateToRelativeLocation rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
                opMode.telemetry.update();

                if (Math.abs(rangeError) <= this.rangeErrorTolerance &&
                        Math.abs(headingError) <= this.headingErrorTolerance &&
                        Math.abs(yawError) <= this.yawErrorTolerance) {
                        opMode.telemetry.addData("Navigate0011", "Success rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
                        opMode.telemetry.update();
                        return Status.SUCCESS;
                }
/////////////////////////////
                drivePower =  -Range.clip(this.rangeController.output(this.currentRelativePositionTarget.range, referenceTag.ftcPose.range), -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);
                turnPower =    Range.clip(-this.headingController.output(this.currentRelativePositionTarget.bearing, referenceTag.ftcPose.bearing), -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);
                strafePower =  Range.clip(-this.yawController.output(this.currentRelativePositionTarget.yaw, referenceTag.ftcPose.yaw), -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);
//////////////////////////////////////////////
                driveTrain.moveRobot(drivePower, strafePower, turnPower);

                return Status.RUNNING;
        }
    private Status navigateToRelativeLocationWithMotionProfile() {
        if (this.referenceTag == null || this.referenceTag.ftcPose == null) {
            return Status.FAILURE;
        }

        setMotionProfiles();

        double rangeError = referenceTag.ftcPose.range - this.currentRelativePositionTarget.range;
        double headingError = referenceTag.ftcPose.bearing - this.currentRelativePositionTarget.bearing;
        double yawError = referenceTag.ftcPose.yaw - this.currentRelativePositionTarget.yaw;
/*
                opMode.telemetry.addData("Navigate1", "SetPoint range: %f heading: %f yaw: %f\n", currentRelativePositionTarget.range, currentRelativePositionTarget.bearing, currentRelativePositionTarget.yaw);
                opMode.telemetry.addData("Navigate1", "navigateToRelativeLocation rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
                opMode.telemetry.update();
*/
        if (Math.abs(rangeError) <= this.rangeErrorTolerance &&
                Math.abs(headingError) <= this.headingErrorTolerance &&
                Math.abs(yawError) <= this.yawErrorTolerance) {
            opMode.telemetry.addData("Navigate0011", "Success rangeError: %f headingError: %f yawError: %f\n", rangeError, headingError, yawError);
            opMode.telemetry.update();
            return Status.SUCCESS;
        }

        // specify the setpoint
        this.rangeFController.setTargetPosition(referenceTag.ftcPose.range);


        double correctionF = this.rangeFController.update(this.currentRelativePositionTarget.range);

        double now = clock.seconds();

        double deltaTime = now - currentStartTime;


        MotionState rangeState = this.rangeMotionProfile.get(deltaTime);

        this.rangeFController.setTargetPosition(rangeState.getX());
        this.rangeFController.setTargetVelocity(rangeState.getV());
        this.rangeFController.setTargetAcceleration(rangeState.getA());

        double driveF = -Range.clip(this.rangeFController.update(this.currentRelativePositionTarget.range), -this.driveTrainConfig.maxAutoSpeed, this.driveTrainConfig.maxAutoSpeed);



        MotionState headingState = this.headingMotionProfile.get(deltaTime);

        this.headingFController.setTargetPosition(headingState.getX());
        this.headingFController.setTargetVelocity(headingState.getV());
        this.headingFController.setTargetAcceleration(headingState.getA());

        double turnF =   Range.clip(this.headingFController.update(this.currentRelativePositionTarget.bearing, referenceTag.ftcPose.bearing), -this.driveTrainConfig.maxAutoTurn, this.driveTrainConfig.maxAutoTurn);


        MotionState yawState = this.yawMotionProfile.get(deltaTime);

        this.yawFController.setTargetPosition(yawState.getX());
        this.yawFController.setTargetVelocity(yawState.getV());
        this.yawFController.setTargetAcceleration(yawState.getA());

        double strafeF = Range.clip(-this.yawFController.update(this.currentRelativePositionTarget.yaw, referenceTag.ftcPose.yaw), -this.driveTrainConfig.maxAutoStrafe, this.driveTrainConfig.maxAutoStrafe);

/*

        opMode.telemetry.addData("Navigate21", " correctionF: %f drive: %f driveF: %f\n" +
                        " deltaTime: %f; state.getX(): %f; state.getV():%f; state.getA():%f\n",
                correctionF, drivePower, driveF,
                deltaTime, rangeState.getX(),rangeState.getV(),rangeState.getA());

        opMode.telemetry.update();

        */


         driveTrain.moveRobot(driveF, strafeF, turnF);

        return Status.RUNNING;
    }

        private Status navigateByTrajectory(GlobalStoreSingleton globalStore){

            Trajectory currentTrajectory = (Trajectory) globalStore.getValue("CurrentTrajectory");
            Pose2d robotStartingPose = (Pose2d) globalStore.getValue("RobotStartingPose");

          //  this.drive.setPoseEstimate(robotStartingPose);
            drive.followTrajectory(currentTrajectory);

            globalStore.setValue("LastRobotPose", drive.getPoseEstimate());

            opMode.telemetry.addData("Navigate navigateByTrajectory", drive.getPoseEstimate().toString());
            opMode.telemetry.update();

            return Status.SUCCESS;
        }

        private void setMotionProfiles(){
               double rangeMaxVelocity = 80;
                double rangeMaxAcceleration = 30;
                double rangeMaxJerk = 20;

                this.rangeMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(initialPose.range, 10, 0),
                        new MotionState(currentRelativePositionTarget.range, 0, 0),
                        rangeMaxVelocity,
                        rangeMaxAcceleration,
                        rangeMaxJerk
                );


                double headingMaxVelocity = 5;
                double headingMaxAcceleration = 1;
                double headingMaxJerk = 5;

                this.headingMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(initialPose.bearing,0,0),
                        new MotionState(0, 0, 0),
                        headingMaxVelocity,
                        headingMaxAcceleration,
                        headingMaxJerk
                );


                double yawMaxVelocity = 5;
                double yawMaxAcceleration = 1;
                double yawMaxJerk = 5;

                this.yawMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(initialPose.yaw, 0, 0),
                        new MotionState(0, 0, 0),
                        yawMaxVelocity,
                        yawMaxAcceleration,
                        yawMaxJerk
                );

                opMode.telemetry.addData("Navigate++++", "rangeMotionProfile: %s \n" ,
                        rangeMotionProfile.getSegments().toString());

                opMode.telemetry.update();
        }
}
