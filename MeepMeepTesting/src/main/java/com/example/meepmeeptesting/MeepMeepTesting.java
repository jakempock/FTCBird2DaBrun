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
                                .splineToSplineHeading(new Pose2d(-17.3, -5.2, Math.toRadians(135)), Math.toRadians(135))
                                .addDisplacementMarker(() -> {})
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-9.5, -13, Math.toRadians(180)), Math.toRadians(-10))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-60, -11.66), Math.toRadians(175))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-14, -11.66, Math.toRadians(-45)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-6, -18), Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(-35, -11.66, Math.toRadians(0)), Math.toRadians(160))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .addEntity(myBot)
                .start();
    }
}