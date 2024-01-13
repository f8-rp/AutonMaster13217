/* Copyright (c) 2021 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="13217 CodoDragons TeleOpVControl")

public class CodoTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;



    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armSwing = null;
    private DcMotor slide = null;
    private DcMotor releaseMotor = null;
    private Servo hand = null;
    private Servo door = null;
    Servo right_hand;
    Servo left_hand;
    Servo release;
    private double increment = 1;
    boolean isSlideDown = true;
    boolean abandoned = true;
    boolean okay = true;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");



        Motor lf = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        Motor rf = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        Motor rb = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);
        Motor lb = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);

        armSwing = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        hand = hardwareMap.get(Servo.class, "hand");
        door = hardwareMap.get(Servo.class, "door");
        release = hardwareMap.get(Servo.class, "release");

        lf.setRunMode(Motor.RunMode.VelocityControl);
        rf.setRunMode(Motor.RunMode.VelocityControl);
        rb.setRunMode(Motor.RunMode.VelocityControl);
        lb.setRunMode(Motor.RunMode.VelocityControl);

        lf.setVeloCoefficients(0.6, 0, 0.01);
        lf.setFeedforwardCoefficients(0.92, 0.47);
        lf.setFeedforwardCoefficients(0.92, 0.47, 0.3);

        rf.setVeloCoefficients(0.6, 0, 0.01);
        rf.setFeedforwardCoefficients(0.92, 0.47);
        rf.setFeedforwardCoefficients(0.92, 0.47, 0.3);

        rb.setVeloCoefficients(0.6, 0, 0.01);
        rb.setFeedforwardCoefficients(0.92, 0.47);
        rb.setFeedforwardCoefficients(0.92, 0.47, 0.3);

        lb.setVeloCoefficients(0.6, 0, 0.01);
        lb.setFeedforwardCoefficients(0.92, 0.47);
        lb.setFeedforwardCoefficients(0.92, 0.47, 0.3);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        class dropOnPresenter implements Runnable{
            public void run(){
                hand.setPosition(0.9);
                sleep(1000);
                armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSwing.setTargetPosition(1490);
                armSwing.setPower(0.6);
                sleep(1500);
                hand.setPosition(0.3);
                sleep(1000);
                armSwing.setTargetPosition(0);
                armSwing.setPower(0.6);
            }
        }

        class slideDown implements Runnable{
            public void run(){
                slide.setPower(1);
                sleep(2000);
                slide.setPower(0);
            }
        }

        class putOnBackdrop implements Runnable{
            public void run(){
                okay = false;
                slide.setPower(-1);
                sleep(1000);
                slide.setPower(-0.01);
                door.setPosition(0.3);
                sleep(1500);
                slide.setPower(1);
                sleep(1100);
                isSlideDown = true;
                okay = true;
            }
        }


        release.setPosition(0.7);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sleep(1000);

        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setTargetPosition(0);
        armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        sleep(500);

        waitForStart();
        runtime.reset();
        double mult = 1;
        int reverse = 1;

        Thread dropOnPresent = new Thread(new dropOnPresenter());
        Thread backdropPlace = new Thread(new putOnBackdrop());
        Thread goDown = new Thread(new slideDown());

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.


            double axial   = -gamepad1.left_stick_y * reverse;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * reverse;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if(gamepad2.left_bumper){
                //macro
                slide.setPower(0.5);
                dropOnPresent.start();
            }

            if(gamepad2.right_bumper){
                //backdrop place
                backdropPlace.start();
            }

            if (gamepad1.y){
                reverse *= -1;
                sleep(300);
            }
            if(gamepad2.left_trigger==1){
                //arm in
                armSwing.setTargetPosition(1350);
                armSwing.setPower(0.6);
            }
            if(gamepad2.right_trigger==1){
                //arm out
                armSwing.setTargetPosition(0);
                armSwing.setPower(0.6);
            }


            if(okay){
                if(isSlideDown == false){
                    if(gamepad2.a){
                        //slide up
                        slide.setPower(-0.75);
                        isSlideDown = false;
                    }
                    else if(gamepad2.y){
                        //slide DOWN
                        goDown.start();
                        isSlideDown = true;
                    }
                    else if(gamepad2.b){
                        //slide down
                        isSlideDown = false;
                        slide.setPower(1);
                    }
                    else{
                        slide.setPower(-0.1);
                    }
                }
                else{
                    if(gamepad2.a){
                        //slide up
                        isSlideDown = false;
                        slide.setPower(-0.75);
                    }
                    if(gamepad2.b){
                        //slide down
                        isSlideDown = false;
                        slide.setPower(0.75);
                    }
                    else{
                        slide.setPower(0.1);
                    }
                }
            }


            if(gamepad2.x){
                armSwing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSwing.setPower(0.4);
                sleep(50);
                armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSwing.setTargetPosition(0);
            }

            if(gamepad2.start){
                armSwing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSwing.setPower(-0.4);
                sleep(50);
                armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSwing.setTargetPosition(0);
            }

            if(gamepad2.dpad_left){
                release.setPosition(1.3);
                sleep(1000);
                release.setPosition(0.7);
            }

            if(okay){
                if(gamepad2.dpad_down){
                    door.setPosition(0.3);
                }
                else if(gamepad2.dpad_right){
                    door.setPosition(0.65);
                }
                else{
                    door.setPosition(1);
                }
            }

            if(gamepad1.a){
                increment = 0.37;
                mult= 0.37;
            }
            if (gamepad1.right_trigger == 1){
                increment = 1;
                mult= 1;
            }
            if(gamepad1.b){
                increment = 0.5;
                mult= 0.5;
            }
            if(gamepad1.x){
                increment = 1.0;
                mult = 1.0;
            }
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            // leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            // leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            // rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            // rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Send calculated power to wheels


            double k = 0.5;



//            leftFrontDrive.setPower(leftFrontPower*increment*1);
//            rightFrontDrive.setPower(rightFrontPower*increment);
//            leftBackDrive.setPower(leftBackPower*increment*1);
//            rightBackDrive.setPower(rightBackPower*increment);
//            lf.set(-(leftFrontPower*increment));
//            rf.set(rightFrontPower*increment);
//            lb.set(-(leftBackPower*increment));
//            rb.set(rightBackPower*increment);
            lf.set(-leftFrontPower * increment);
            rf.set(rightFrontPower);
            lb.set(-leftBackPower * increment);
            rb.set(rightBackPower);

            // Show the elapsed game time and wheel power.*/
            telemetry.addData("encoder position", armSwing.getCurrentPosition());
            telemetry.update();
        }
    }}
