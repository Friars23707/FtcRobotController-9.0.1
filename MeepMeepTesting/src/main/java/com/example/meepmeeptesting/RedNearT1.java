package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedNearT1 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                                .back(25)
                                .strafeRight(10)
                                .strafeLeft(10)
                                .turn(Math.toRadians(-90))
                                .forward(3)
                                .addDisplacementMarker(() -> {
                                    //DROP PURPLE PIXEL
                                })
                                .waitSeconds(1)
                                .back(3)
                                .strafeRight(22)
                                .back(40)
                                .strafeLeft(15)
                                .addDisplacementMarker(() -> {
                                    //DROP YELLOW PIXEL
                                })
                                .waitSeconds(1)
                                .strafeRight(20)
                                .back(10)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
