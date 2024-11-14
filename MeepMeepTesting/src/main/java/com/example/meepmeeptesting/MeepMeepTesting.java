package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueChamber;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Trajectory builders for Blue and Red alliances when scoring on the chambers or in the baskets
        TrajectoryActionBuilder actionBuilder = myBot.getDrive().actionBuilder(new Pose2d(BlueChamber.INITIAL_POSE_X, BlueChamber.INITIAL_POSE_Y, BlueChamber.INITIAL_HEADING));
        BlueChamber trajectoryBuilder = new BlueChamber(myBot.getDrive().actionBuilder(new Pose2d(BlueChamber.INITIAL_POSE_X, BlueChamber.INITIAL_POSE_Y, BlueChamber.INITIAL_HEADING)));

        myBot.runAction(trajectoryBuilder.getTrajectory());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
