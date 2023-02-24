package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.48180821614297,
                        52.48180821614297,
                        Math.toRadians(229.2849698332325),
                        Math.toRadians(184.02607784577722),
                        12.72)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-30, -62, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-26, -61), Math.toRadians(5))
                                .splineToConstantHeading(new Vector2d(-11.66, -16), Math.toRadians(95))
                                .splineToSplineHeading(new Pose2d(-14.7, -6.75, Math.toRadians(135)), Math.toRadians(135))
                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-9.5, -13, Math.toRadians(180)), Math.toRadians(-10))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-55.3, -11), Math.toRadians(175))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-35.5, -11.66, Math.toRadians(-140)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-45.8, -10.3, Math.toRadians(-90)), Math.toRadians(-100))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .addEntity(myBot)
                .start();
    }
}