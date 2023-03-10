/* Copyright (c) 3017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "RedRightAuto", group = "Iterative Opmode")

public class RedRightAuto extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;

    private int camViewId;
    private AprilTagDetectionPipeline detectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C930 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.456;

    // UNITS ARE METERS
    double tagsize = 0.166;

    private int detectionId = -1;
    private boolean parking = false;
    private final Pose2d START_POSE = new Pose2d(-30, -62, Math.toRadians(90));
    private DcMotor arm;
    private final double ARM_TICKS_PER_REV = 384.5;
    private Servo armServo;
    private enum State {
        CLOSE_CLAW_START,
        LIFT_ARM_SMALL,
        TO_POLE_FIRST,
        DROP_CONE_FIRST,
        DROP_ARM,
        TO_STACK,
        CLOSE_CLAW_STACK,
        LIFT_ARM_STACK,
        TO_POLE_SECOND,
        DROP_CONE_SECOND,
        PARK,
        END
    }
    State currState;
    long waitEnd;
    private TrajectorySequence toPoleFirst;
    private TrajectorySequence toStack;
    private TrajectorySequence toPoleSecond;
    private TrajectorySequence leftPark, midPark, rightPark;

    private void doNextState() {
        if (System.currentTimeMillis() < waitEnd || drive.isBusy()) {
           return;
        }
        switch (currState) {
            case CLOSE_CLAW_START:
                armServo.setPosition(0.259);
                waitEnd = System.currentTimeMillis() + 500;
                currState = State.LIFT_ARM_SMALL;
                break;
            case LIFT_ARM_SMALL:
                arm.setTargetPosition((int) (ARM_TICKS_PER_REV * 2));
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                waitEnd = System.currentTimeMillis() + 2000;
                currState = State.TO_POLE_FIRST;
                break;
            case TO_POLE_FIRST:
                drive.followTrajectorySequenceAsync(toPoleFirst);
                waitEnd = System.currentTimeMillis() + 1000;
                currState = State.DROP_CONE_FIRST;
                break;
            case DROP_CONE_FIRST:
                armServo.setPosition(0);
                waitEnd = System.currentTimeMillis() + 500;
                currState = State.DROP_ARM;
                break;
            case DROP_ARM:
                arm.setPower(0.6);
                arm.setTargetPosition((int) (ARM_TICKS_PER_REV * 1.15));
                currState = State.TO_STACK;
            case TO_STACK:
                drive.followTrajectorySequenceAsync(toStack);
                currState = State.CLOSE_CLAW_STACK;
                break;
            case CLOSE_CLAW_STACK:
                armServo.setPosition(0.259);
                currState = State.LIFT_ARM_STACK;
                waitEnd = System.currentTimeMillis() + 500;
                break;
            case LIFT_ARM_STACK:
                // arm.setPower(1);
                arm.setTargetPosition((int) (ARM_TICKS_PER_REV * 3.5));
                waitEnd = System.currentTimeMillis() + 1000;
                currState = State.TO_POLE_SECOND;
                break;
            case TO_POLE_SECOND:
                drive.followTrajectorySequenceAsync(toPoleSecond);
                currState = State.DROP_CONE_SECOND;
                break;
            case DROP_CONE_SECOND:
                armServo.setPosition(0);
                waitEnd = System.currentTimeMillis() + 500;
                currState = State.PARK;
                break;
            case PARK:
                switch (detectionId) {
                    case 1: drive.followTrajectorySequenceAsync(leftPark); break;
                    case 2: drive.followTrajectorySequenceAsync(midPark); break;
                    case 3: drive.followTrajectorySequenceAsync(rightPark); break;
                }
                currState = State.END;
                break;
            case END:
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = hardwareMap.get(DcMotor.class, "armMotor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(1);
        armServo = hardwareMap.get(Servo.class, "armServo");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        waitEnd = 0;
        currState = State.CLOSE_CLAW_START;
        toPoleFirst = drive
                .trajectorySequenceBuilder(START_POSE)
                .splineToConstantHeading(
                        new Vector2d(-22, -61),
                        Math.toRadians(10),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(() -> arm.setTargetPosition((int) (ARM_TICKS_PER_REV * 7.7)))
                .splineToConstantHeading(
                        new Vector2d(-11.66, -16),
                        Math.toRadians(95),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-17.4, -4.8, Math.toRadians(135)),
                        Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        toStack = drive
                .trajectorySequenceBuilder(toPoleFirst.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-9.5, -13, Math.toRadians(180)), Math.toRadians(-10))
                .setReversed(false)
                .splineToConstantHeading(
                        new Vector2d(-59.5, -11.66),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        toPoleSecond = drive
                .trajectorySequenceBuilder(toStack.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-14, -11.66, Math.toRadians(-45)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-6, -18), Math.toRadians(-45))
                .build();
        leftPark = drive
                .trajectorySequenceBuilder(toPoleSecond.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-11.66, -11.66, Math.toRadians(0)), Math.toRadians(180))
                .build();
        midPark = drive
                .trajectorySequenceBuilder(toPoleSecond.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35, -11.66, Math.toRadians(0)), Math.toRadians(160))
                .build();
        rightPark = drive
                .trajectorySequenceBuilder(toPoleSecond.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -11.66, Math.toRadians(20)), Math.toRadians(160))
                .build();

        camViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        telemetry.addData("Camera Viewport Id", camViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        telemetry.addData("name: ", webcamName.toString());
        OpenCvCamera camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(webcamName, camViewId);

        detectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(detectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed to open camera", errorCode);
            }
        });

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("detectionid", detectionId);
        telemetry.addData("running", parking);
        telemetry.addData("isBusy", drive.isBusy());
        drive.update();
        if (detectionId == -1) {
            ArrayList<AprilTagDetection> detections = detectionPipeline.getLatestDetections();
            switch (detections.size()) {
                case 0:
                    telemetry.addData("detections", "none");
                    break;
                case 1:
                    telemetry.addData("detections", detections.get(0).id);
                    detectionId = detections.get(0).id;
                    break;
                default:
                    telemetry.addData("detections", "more than one! assuming first " + detections.get(0).id);
                    detectionId = detections.get(0).id;
                    break;
            }
        } else {
            doNextState();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
