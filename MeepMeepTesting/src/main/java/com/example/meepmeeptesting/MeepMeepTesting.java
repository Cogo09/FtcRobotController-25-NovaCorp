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
//!RED AUTONOMOUS BELOW SCORES THEORETICAL 18 Ball and clear
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, 10, 0))
                            .turnTo(Math.toRadians(155))
                            .waitSeconds(2)
                    //preloads
                            .turnTo(Math.toRadians(180))
                            .lineToX(35)
                            .turnTo(Math.toRadians(90))
                            .lineToY(40)
                            .lineToY(10)
                            .turnTo(Math.toRadians(180))
                            .lineToX(-15)
                            .turnTo(Math.toRadians(135))
                            .waitSeconds(2)
                    //first
                            .turnTo(Math.toRadians(180))
                            .lineToX(12)
                            .turnTo(Math.toRadians(90))
                            .lineToY(40)
                            .lineToY(10)
                            .turnTo(Math.toRadians(180))
                            .lineToX(-15)
                            .turnTo(Math.toRadians(135))
                            .waitSeconds(2)
                    //second
                            .turnTo(Math.toRadians(180))
                            .lineToX(0)
                            .turnTo(Math.toRadians(270))
                            .lineToY(54)
                            .waitSeconds(2)
                    //clear stack
                            //.splineTo(new Vector2d(56,56),1)
//                            .lineToY(10)
//                            .turnTo(Math.toRadians(180))
                            .lineToY(10)
                            .turnTo(Math.toRadians(180))
                            .lineToX(-12)
                            .turnTo(Math.toRadians(90))
                            .lineToY(40)
                            .lineToY(10)
                            .turnTo(Math.toRadians(135))
                    //third

                            .splineTo(new Vector2d(56,56),1)
                            .turnTo(Math.toRadians(180))
                    //set
                    //end of auto

                    //new auto





                    .build());
            //!BLUE AUTO Below clears 18 balls and clears
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, -10, 0))
                    .turnTo(Math.toRadians(205))
                    .waitSeconds(2)
                    //preloads
                    .turnTo(Math.toRadians(180))
                    .lineToX(35)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-40)
                    .lineToY(-10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    //first
                    .turnTo(Math.toRadians(180))
                    .lineToX(12)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-40)
                    .lineToY(-10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    //second
                    .turnTo(Math.toRadians(180))
                    .lineToX(0)
                    .turnTo(Math.toRadians(90))
                    .lineToY(-54)
                    .waitSeconds(2)
                    //clear stack
                    //.splineTo(new Vector2d(56,56),1)
//                            .lineToY(10)
//                            .turnTo(Math.toRadians(180))
                    .lineToY(-10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-12)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-40)
                    .lineToY(-10)
                    .turnTo(Math.toRadians(225))
                    //third

                    .splineTo(new Vector2d(56,-56),1)
                    .turnTo(Math.toRadians(270))

                    .build());
            //!RED SHOOT PRELOADS
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, 10, 0))
                            .turnTo(Math.toRadians(155))
                            .waitSeconds(2)
                            .turnTo(Math.toRadians(180))
                            .build());
            //!BLUE SHOOT PRELOADS
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, -10, 0))
                    .turnTo(Math.toRadians(205))
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(180))
                    .build());
            //!BLUE SHOOT 12 clear
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, -10, 0))
                    .turnTo(Math.toRadians(205))
                    .waitSeconds(2)
                    //preloads
                    .turnTo(Math.toRadians(180))
                    .lineToX(35)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-40)
                    .lineToY(-10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    //first
                    .turnTo(Math.toRadians(180))
                    .lineToX(12)
                    .turnTo(Math.toRadians(270))
                    .lineToY(-40)
                    .lineToY(-10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    //second
                    .turnTo(Math.toRadians(180))
                    .lineToX(0)
                    .turnTo(Math.toRadians(90))
                    .lineToY(-54)
                    .waitSeconds(2)
                    .build());
            //!RED SHOOT 12 clear
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, 10, 0))
                    .turnTo(Math.toRadians(155))
                    .waitSeconds(2)
                    //preloads
                    .turnTo(Math.toRadians(180))
                    .lineToX(35)
                    .turnTo(Math.toRadians(90))
                    .lineToY(40)
                    .lineToY(10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(2)
                    //first
                    .turnTo(Math.toRadians(180))
                    .lineToX(12)
                    .turnTo(Math.toRadians(90))
                    .lineToY(40)
                    .lineToY(10)
                    .turnTo(Math.toRadians(180))
                    .lineToX(-15)
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(2)
                    //second
                    .turnTo(Math.toRadians(180))
                    .lineToX(0)
                    .turnTo(Math.toRadians(270))
                    .lineToY(54)
                    .waitSeconds(2)
                    .build());



            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
