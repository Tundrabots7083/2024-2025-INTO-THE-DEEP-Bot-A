package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

public class NavigateAndOrientToSample implements ActionFunction {

    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KPDrive = 0.008;
    public static double KIDrive = 0.007;
    public static double KDDrive = 0;
    public static double KPTurn = 0.025;
    public static double KITurn = 0.05;
    public static double KDTurn = 0;
    public static double TOLERABLE_ERROR = 0.5; // inches
    public static double MAXIMUM_INTAKE_DISTANCE = 35; //inches
    public static double INTAKE_ANGLE = 0.0;

    private final PIDControllerImpl DrivePIDController;
    private final PIDControllerImpl TurnPIDController;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public NavigateAndOrientToSample(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        DrivePIDController = new PIDControllerImpl(KPDrive, KIDrive, KDDrive);
        DrivePIDController.reset();

        TurnPIDController = new PIDControllerImpl(KPTurn,KITurn,KDTurn);
        TurnPIDController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double xDistanceToSample;
        double distanceError;
        double turnError;
        double drivePower = 0.0;
        double turnPower = 0.0;

        if (lastStatus == Status.SUCCESS) {
            telemetry.addData("[NavigateWithinRange] status:", "Just left it as success");
            telemetry.update();
            return lastStatus;
        }


        if (blackBoard.getValue("xDistanceToSample") != null) {
            xDistanceToSample = (double) blackBoard.getValue("xDistanceToSample");
            distanceError = xDistanceToSample - MAXIMUM_INTAKE_DISTANCE;
            telemetry.addData("[Navigate]distance",xDistanceToSample);
            telemetry.addData("[Navigate]distanceError",distanceError);
            telemetry.update();
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsInIntakeRange",false);
            return status;
        }

        if (blackBoard.getValue("Tx") != null) {
            turnError = (double) blackBoard.getValue("Tx");
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsSquaredWithSample",false);
            return status;
        }


        if (distanceError <= TOLERABLE_ERROR && Math.abs(turnError) <= TOLERABLE_ERROR) {
            mecanumDrive.driveStraight(0.0);
            status = Status.SUCCESS;
            blackBoard.setValue("BotIsInIntakeRange",true);
            blackBoard.setValue("BotIsSquaredWithSample", true);
            lastStatus = status;
            return status;
        }

        if (distanceError >= TOLERABLE_ERROR) {
            drivePower = DrivePIDController.calculate(MAXIMUM_INTAKE_DISTANCE, distanceError);
        }
        if (Math.abs(turnError) >= TOLERABLE_ERROR) {
            turnPower = TurnPIDController.calculate(INTAKE_ANGLE, turnError);
        }

        mecanumDrive.drive(0,drivePower,turnPower);

        runCount++;

        telemetry.addData("[NavigateWithinRange] status:", status);
        telemetry.update();
        lastStatus = status;

        return status;
    }
}
