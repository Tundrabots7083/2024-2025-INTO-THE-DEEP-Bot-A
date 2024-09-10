package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Arm is the arm and extension that are used to move the scoring mechanism to the correct location.
 */
@Config
public class Arm implements Subsystem {
    public static int LOW_ARM_POSITION = 2975;
    public static int MEDIUM_ARM_POSITION = 2815;
    public static int HIGH_ARM_POSITION = 2600;
    public static double LOW_SERVO_POSITION = 0.30;
    public static double MEDIUM_SERVO_POSITION = 0.30;
    public static double HIGH_SERVO_POSITION = 0.30;
    // PID control constants. TODO: change to private final once tuned
    public static double ARM_KP = 0.0053;
    public static double ARM_KI = 0.01;
    public static double ARM_KD = 0.00013;
    public static double LIFT_KP = 0.0053;
    public static double LIFT_KI = 0.01;
    public static double LIFT_KD = 0.00013;
    public static double INTEGRAL_LIMIT = 1;
    public static int TOLERABLE_ERROR = 20;
    public static double POWER_CAP = 0.9;
    public static double MIN_POWER = 0.1;
    private final Telemetry telemetry;
    private final Motor armMotor;
    private final Motor extensionMotor;
    private PIDController armController;
    private PIDController extensionController;
    private Position target = Position.Start;

    /**
     * Creates a new scoring arm with the given device name and description.
     *
     * @param hardwareMap the hardware found on the robot
     * @param telemetry   telemetry used to display data on the driver station and FTC dashboard
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = new Motor(hardwareMap, "arm");
        extensionMotor = new Motor(hardwareMap, "extension");

        initMotor(armMotor);
        initMotor(extensionMotor);

        armController = new PIDController(ARM_KP, ARM_KI, ARM_KD);
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        extensionController = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        extensionController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    /**
     * Initializes the arm motor.
     *
     * @param motor the arm motor to be initialized.
     */
    private void initMotor(Motor motor) {
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Motor.Direction.FORWARD);
    }

    /**
     * Returns an indication as to whether the arm and extension are at their target positions.
     *
     * @return <code>true</code> if both the arm and extension are at their target positions;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        return isArmAtTarget() && isExtensionAtTarget();
    }

    private boolean isArmAtTarget() {
        int armPos = armMotor.getCurrentPosition();
        return Math.abs(target.armPosition - armPos) <= TOLERABLE_ERROR;
    }

    private boolean isExtensionAtTarget() {
        int extensionPos = extensionMotor.getCurrentPosition();
        return Math.abs(target.extensionPosition - extensionPos) <= TOLERABLE_ERROR;
    }

    /**
     * Sets the target position for the arm.
     *
     * @param target the target position of the arm.
     */
    public void setTarget(Position target) {
        if (this.target != target) {
            resetPIDControllers();
            this.target = target;
        }
    }

    public void stopArm() {
        armMotor.setPower(0);
    }

    public void setArmPower(double power) {
        power = Range.clip(power, -1, 1);
        armMotor.setPower(power);
    }

    /**
     * Update sets the power for the arm motor based on the target position.
     */
    public void update() {
        // Short-circuit the processing if the arm is at it's target
        if (isAtTarget()) {
            stopArm();
            telemetry.addLine("[ARM] At Target");
            return;
        }

        double armPower = calculate(armController, target.armPosition, armMotor.getCurrentPosition());
        armMotor.setPower(armPower);
        double extensionPower = calculate(extensionController, target.extensionPosition, extensionMotor.getCurrentPosition());
        extensionMotor.setPower(extensionPower);
        telemetry.addData("[ARM] Arm Power", armPower);
        telemetry.addData("[ARM] Extension Power", extensionPower);
    }

    /**
     * Calculates the power to apply to the arm motor.
     *
     * @return the power to apply to the motor.
     */
    private double calculate(PIDController controller, double target, double current) {
        double power = -controller.calculate(target, current);
        power = Range.clip(power, -POWER_CAP, POWER_CAP);
        if (power > 0.0 && power < MIN_POWER) {
            power = MIN_POWER;
        } else if (power < 0.0 && power > -MIN_POWER) {
            power = -MIN_POWER;
        }

        return power;
    }

    public void start() {
        target = Position.Intake;
        update();
    }

    /**
     * resetPIDControllers resets the PID control values.
     */
    private void resetPIDControllers() {
        armController = new PIDController(ARM_KP, ARM_KI, ARM_KD);
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        extensionController = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        extensionController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    /**
     * Position defines the position of the arm and pixel container.
     */
    public enum Position {
        Start(0, 0, 0),
        Intake(0, 0, 0.65),
        ScoreLow(LOW_ARM_POSITION, 0, LOW_SERVO_POSITION),
        ScoreMedium(MEDIUM_ARM_POSITION, 0, MEDIUM_SERVO_POSITION),
        ScoreHigh(HIGH_ARM_POSITION, 0, HIGH_SERVO_POSITION),
        Hang(2000, 0, 0.65),
        LaunchDrone(1505, 0, 1.0);

        public final int armPosition;
        public final int extensionPosition;
        public final double servoPosition;

        /**
         * Creates a new Position for the given arm and servo.
         *
         * @param armPosition   the position of the arm.
         * @param extensionPosition  the position of the extension.
         * @param servoPosition the position of the flip servo.
         */
        Position(int armPosition, int extensionPosition, double servoPosition) {
            this.armPosition = armPosition;
            this.extensionPosition = extensionPosition;
            this.servoPosition = servoPosition;
        }
    }
}