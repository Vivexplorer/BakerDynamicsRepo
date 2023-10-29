package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class LeftSpikeTestingRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
//                                .back(5)
//                                .turn(Math.toRadians(90))
//                                .strafeLeft(22)
//                                .forward(5)
                                .lineTo(new Vector2d(24, -42))
                                .splineTo(new Vector2d(10, -37), Math.toRadians(140))
                                .forward(10)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(40,-29),Math.toRadians(0))
                                .back(8)
                                .strafeLeft(30)
                                .back(10)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
