package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 55
                        , Math.toRadians(270)))
                //.splineTo(new Vector2d(-8, 24), Math.toRadians(230))
                //.splineTo(new Vector2d(-48, 45), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(-35, 38), Math.toRadians(245))
                .strafeToLinearHeading(new Vector2d(-45, 47), Math.toRadians(125))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-41, 12), Math.toRadians(265))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
