package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                                .setReversed(false)
                                .strafeTo(new Vector2d(25, 32))
                                .turn(Math.toRadians(90))
                                .forward(18)
                                .addDisplacementMarker(() -> {
                                   /* try {
                                        spikeDrop();
                                    } catch (InterruptedException e) {
                                        throw new RuntimeException(e);
                                    }*/
                                })
                                .strafeTo(new Vector2d(42, 28))
                                .addDisplacementMarker(() -> {
                                    /*try {
                                        boardDrop();
                                    } catch (InterruptedException e) {
                                        throw new RuntimeException(e);
                                    }*/
                                })
                                .strafeLeft(18)
                                .back(12)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}