package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;

/**
 * This class implements sequences of actions that are needed
 * to score samples and specimens.
 */
@Config
public class ScoringSubsystem extends SubsystemBase {
    // Heights of scoring places are in inches
    public static double HIGH_CHAMBER_HEIGHT = 26.0;
    public static double LOW_CHAMBER_HEIGHT = 13.0;
    public static double HIGH_BASKET_HEIGHT = 43.0;
    public static double LOW_BASKET_HEIGHT = 25.75;
    // Heights scoring subsystem needs to reach to score in
    // different places in inches
    public static double HIGH_CHAMBER_SCORING_HEIGHT = 27.0;
    public static double HIGH_BASKET_SCORING_HEIGHT = 44.0;
    // This is the length of the arm and the wrist
    // with zero extension.
    public static double ARM_LENGTH = 21.0;
    // This is the height from the field to the thing
    // that the arm rotates around.
    // todo:  Need to find out what the word for the thing the arm
    //        rotates on is called.
    public static double ARM_HEIGHT = 11.0;
    // This is the distance from the front of the robot to the back
    // of the robot.
    public static double ROBOT_LENGTH = 18.0;

    Arm arm;
    LinearSlide slide;
    Wrist wrist;
    Claw claw;

    boolean settingChamberHighPosition = false;
    boolean settingBasketHighPosition = false;

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high chamber bar.
     */
    void setChamberHighPosition() {

    }

    /**
     * Moves the arm, slide, wrist, and claw to score a specimen
     * on either the high or low chamber bar.
     */
    void scoreSpecimen() {

    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high basket.
     */
    void setBasketHighPosition() {

    }

    /**
     * Moves the arm, slide, wrist, and claw to score a sample
     * anywhere (high and low baskets and floor).
     */
    void scoreSample() {
        // Samples can be scored in a basket or on the floor
        // Need to open claw to release the sample.
    }
}
