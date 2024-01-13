package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Autonomous(name="Auton Red Stack Side", group="Autonomous")

public final class AutonMasterRed extends LinearOpMode {
    private DcMotorEx armSwing = null;
    private Servo hand;
    private DcMotor slide = null;

    private Servo door;

    @Override
    public void runOpMode() {


        VisionPortal.Builder build = new VisionPortal.Builder();
        build.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        armSwing = hardwareMap.get(DcMotorEx.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        door = hardwareMap.get(Servo.class, "door");

        hand.setPosition(0.9);

       sleep(2000);

        final double DELAY = 0;

        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setTargetPosition(1500);
        armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSwing.setPower(0.4);

        door.setPosition(1);

        OurRobotRed myRobot = new OurRobotRed(build);

        String teamPropLocation = new String("notFound");
        while (!isStarted() && !isStopRequested()) {
            String newPropLoc = myRobot.telemetryTfod();
            if (!newPropLoc.equals("notFound")){
                teamPropLocation = newPropLoc;
            }
            telemetry.addData("team prop location: ", teamPropLocation);
            telemetry.addData("go to: ", teamPropLocation);
            telemetry.update();
        }
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-33, -62.5, Math.toRadians(90)));


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            waitForStart();

            if (teamPropLocation == "left") {
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(DELAY),
                                        spinDown(),
                                        new SleepAction(2),
                                        servoOpen(),
                                        new SleepAction(8),
                                        slideUp(),
                                        new SleepAction(1.5),
                                        doorDown(),
                                        new SleepAction(2),
                                        slideDown(),
                                        doorUp()
                                ),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .waitSeconds(DELAY+1)
                                                //-34 64
                                                .strafeToLinearHeading(new Vector2d(-47, -50), Math.toRadians(90))
                                                .waitSeconds(2)
                                                .strafeToLinearHeading(new Vector2d(-34, -37), Math.toRadians(180))
                                                .strafeToLinearHeading(new Vector2d(-37, -60.5), Math.toRadians(180)) // get back
                                                .strafeToLinearHeading(new Vector2d(48, -60.5), Math.toRadians(180)) // get back to board
                                                .strafeToLinearHeading(new Vector2d(52,-33), Math.toRadians(180))
                                                .strafeToLinearHeading(new Vector2d(49,-33), Math.toRadians(180))
                                                .waitSeconds(4)
                                                .strafeToLinearHeading(new Vector2d(46,-33), Math.toRadians(180))
                                                .build()
                                )
                        )
                );
            }

            //asdf
            if (teamPropLocation.equals("notFound") || teamPropLocation == "middle") {
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(DELAY),
                                        spinDown(),
                                        new SleepAction(0.5),
                                        servoOpen(),
                                        new SleepAction(8),
                                        slideUp(),
                                        new SleepAction(1.5),
                                        doorDown(),
                                        new SleepAction(2),
                                        slideDown(),
                                        doorUp(),
                                        new SleepAction(8),
                                        servoClose(),
                                        new SleepAction(2),
                                        spinUp(),
                                        new SleepAction(1),
                                        servoOpen(),
                                        new SleepAction(0.5),
                                        spinDown()
                                ),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .waitSeconds(DELAY+1)
                                                .strafeToLinearHeading(new Vector2d(-34, -40), Math.toRadians(90))
                                                .waitSeconds(1)
                                                .turn(Math.toRadians(90))
                                                .strafeToLinearHeading(new Vector2d(-33, -60.5), Math.toRadians(180)) // get back
                                                .strafeToLinearHeading(new Vector2d(48, -60.5), Math.toRadians(180)) // get back to board
                                                .strafeToLinearHeading(new Vector2d(49.4,-40), Math.toRadians(180))
                                                .waitSeconds(4)
                                                .strafeToLinearHeading(new Vector2d(45,-40), Math.toRadians(180))
                                                .build()
//
                                )
                        )
                ));
            }

            if (teamPropLocation == "right") {
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(DELAY),
                                        spinDown(),
                                        new SleepAction(5),
                                        servoOpen(),
                                        new SleepAction(10),
                                        slideUp(),
                                        new SleepAction(1.5),
                                        doorDown(),
                                        new SleepAction(2),
                                        slideDown(),
                                        doorUp()
                                ),
                                new SequentialAction(
                                        drive.actionBuilder(drive.pose)
                                                .waitSeconds(DELAY+1)
                                                .strafeToLinearHeading(new Vector2d(-40, -35), Math.toRadians(90)) //up
                                                .waitSeconds(2)
                                                .turn(Math.toRadians(-90))
                                                .waitSeconds(2)
                                                .turn(Math.toRadians(180))
                                                .strafeToLinearHeading(new Vector2d(-33, -60.5), Math.toRadians(180)) // get back
                                                .strafeToLinearHeading(new Vector2d(48, -60.5), Math.toRadians(180)) // get back to board
                                                .strafeToLinearHeading(new Vector2d(51, -45), Math.toRadians(180))
                                                .waitSeconds(4)
                                                .strafeToLinearHeading(new Vector2d(47, -45), Math.toRadians(180))
                                                .build()
                                )
                        )
                );
            }
        }
    }


    public Action doorDown() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    door.setPosition(0.3);
                    initialized = true;
                }

                if(door.getPosition()==0.3){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action doorUp() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    door.setPosition(1);
                    initialized = true;
                }

                if(door.getPosition()>0){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }
    public Action slideUp() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(-1);
                    sleep(600);
                    slide.setPower(-0.1);

                    initialized = true;
                }

                if(slide.getPower()==-0.1){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action slideDown() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(1);
                    sleep(600);
                    slide.setPower(0);
                    initialized = true;
                }

                if(slide.getPower()==0){
                    return false;
                }
                else{
                    return true;
                }

            }
        };
    }

    public Action spinUp() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armSwing.setTargetPosition(1580);
                    armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armSwing.setPower(0.4);
                    initialized = true;
                }

                if(armSwing.getCurrentPosition() >= 1575){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action spinUpHalf() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armSwing.setTargetPosition(50);
                    armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armSwing.setPower(0.4);
                    initialized = true;
                }

                if(armSwing.getCurrentPosition() >= 45){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action servoOpen() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    hand.setPosition(0.6);

                    initialized = true;
                }

                if(hand.getPosition() == 0.6){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action servoClose() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    hand.setPosition(0.9);

                    initialized = true;
                }

                if(hand.getPosition() == 0.9){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }

    public Action spinDown() {
        return new Action() {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armSwing.setTargetPosition(80);
                    armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armSwing.setPower(0.4);
                    initialized = true;
                }

                if(armSwing.getCurrentPosition() <= 85){
                    return false;
                }
                else{
                    return true;
                }
            }
        };
    }
}


//TODO: clean up useless comments and remove extra redundancy that may cause bugs

class OurRobotRed{
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;
    private DcMotor arm = null;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/D1 FTC-ML Hater (red).tflite";
    private static final String[] LABELS = {"prop"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private VisionPortal.Builder builder;


    //This constructor takes in the motors which are of class DcMotor
    public OurRobotRed(VisionPortal.Builder build1){
        builder = build1;
        initTfod();
    }


    //Tensorflow Object Detection functions:
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    public String telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            if (x <= 328) {
                return "left";
            }
            else {
                return "right";
            }
        }
        return "notFound";
    }
}
