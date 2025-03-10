package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-21, -66, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -27), Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -50))
                .splineToConstantHeading(new Vector2d(38, -16), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(90))
                .strafeTo(new Vector2d(48, -70))
                .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(90))
                .strafeTo(new Vector2d(58, -16))
                .strafeTo(new Vector2d(58, -70))
                .strafeTo(new Vector2d(58, -16))
                .splineToConstantHeading(new Vector2d(68, -12), Math.toRadians(90))
                .strafeTo(new Vector2d(68, -70))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}