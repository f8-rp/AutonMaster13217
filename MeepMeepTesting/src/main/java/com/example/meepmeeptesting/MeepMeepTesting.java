package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 55, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -63, Math.toRadians(90)))


                //blue stack, left
//                .splineTo(new Vector2d(-33, 42), Math.toRadians(315))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-40, 61), Math.toRadians(180)) //get back
//                .strafeToLinearHeading(new Vector2d(35,59.5), Math.toRadians(180)) //get to board
//                .strafeToLinearHeading(new Vector2d(54,36.2), Math.toRadians(180))

                //blue stack, right
//                .strafeToLinearHeading(new Vector2d(-34, 34), Math.toRadians(270))
//                .turn(Math.toRadians(-90))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-34, 37), Math.toRadians(180))
//                .waitSeconds(1)
//
//                .strafeToLinearHeading(new Vector2d(-27.5, 37), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-34, 34), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-40, 61), Math.toRadians(180)) //get back
//                .strafeToLinearHeading(new Vector2d(35,59.5), Math.toRadians(180)) //get to board
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(54,36), Math.toRadians(180))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(53.5,26), Math.toRadians(180))

//                //blue stack, middle
//                .strafeToLinearHeading(new Vector2d(-38, 42), Math.toRadians(270))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-40, 61), Math.toRadians(180)) //get back
//                .strafeToLinearHeading(new Vector2d(35,59.5), Math.toRadians(180)) //get to board
//                .strafeToLinearHeading(new Vector2d(40,35.6), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(54.5,32.6), Math.toRadians(180))


//                .strafeToLinearHeading(new Vector2d(-40, -35), Math.toRadians(90)) //up
//                        .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-33, -58.5), Math.toRadians(180)) // get back
//                .strafeToLinearHeading(new Vector2d(50, -58.5), Math.toRadians(180)) // get back to board
//                .strafeToLinearHeading(new Vector2d(50, -45), Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(52, -46.5), Math.toRadians(180))


                // red stack middle
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-34, -40), Math.toRadians(90))
//                .waitSeconds(1)
//                .turn(Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-33, -58.5), Math.toRadians(180)) // get back
//                .strafeToLinearHeading(new Vector2d(50, -58.5), Math.toRadians(180)) // get back to board
//                .strafeToLinearHeading(new Vector2d(56,-40), Math.toRadians(180))

                //red stack left
//                .strafeToLinearHeading(new Vector2d(-45, -50), Math.toRadians(90))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-34, -37), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-33, -58.5), Math.toRadians(180)) // get back
//                .strafeToLinearHeading(new Vector2d(50, -58.5), Math.toRadians(180)) // get back to board
//                .strafeToLinearHeading(new Vector2d(56,-34), Math.toRadians(180))


                .strafeToLinearHeading(new Vector2d(-40, -35), Math.toRadians(90)) //up
                .waitSeconds(2)
                .turn(Math.toRadians(-90))
                .waitSeconds(5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}