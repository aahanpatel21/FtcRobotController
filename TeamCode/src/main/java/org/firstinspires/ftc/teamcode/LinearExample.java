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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/*
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Linear Example", group="Linear OpMode")

public class LinearExample extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armRotator = null;
    private DcMotor extendableArm = null;
    private ServoImplEx intake = null;
    private ServoImplEx wrist = null;

    public void setArmTargetPositionDegrees(double angle) {
        // 360 degrees = 1425.1 * 5 ticks
        // 90 degrees = 1425.1 * 5 / (360 / 90) ticks
        armRotator.setTargetPosition((int) (1425.1 * 5 / (360 / angle)));
    }

    public void setWristTargetPositionDegrees(double angle) {
        double angleOffsetFront = 60;
        double angleOffsetBackward = 30;
        double angleMax = 270-angleOffsetBackward-angleOffsetFront;
        if (angle > (angleMax)) {
            angle = angleMax;
        }
        if (angle < 0) {
            angle = 0;
        }
        wrist.setPosition(angleOffsetFront/270 + (angle)/270);
    }
    public void setIntakeTargetPositionDegrees(double angle) {
        double angleOffsetFront = 60;
        double angleOffsetBackward = 150;
        double angleMax = 270-angleOffsetBackward-angleOffsetFront;
        if (angle > (angleMax)) {
            angle = angleMax;
        }
        if (angle < 0) {
            angle = 0;
        }
        intake.setPosition(angleOffsetFront/270 + (angle)/270);
    }
    public void setArmExtensionTargetPositionMM(double length) {
        // 360 degrees = 537.7 ticks
        // 60t pulley - 2mm pitch
        // 360 degrees = 60 * 2mm = 120mm extension
        // Hard limit is 650mm

        if (length > 675) {
            length = 675;
        }

        extendableArm.setTargetPosition((int)(537.7 * length/120));
    }

    public void wristreverse() {
        wrist.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
    }

    public void wristforward() {
        wrist.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
    }
    @Override
    public void runOpMode() {
        int extendableArmLimit = 4500;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "m3");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "m2");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "m0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "m1");
        armRotator = hardwareMap.get(DcMotor.class, "m4");
        extendableArm = hardwareMap.get(DcMotor.class, "m5");
        intake = hardwareMap.get(ServoImplEx.class,  "s1");
        wrist = hardwareMap.get(ServoImplEx.class,  "s2");
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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.REVERSE);
        wristforward();

        // Run until the end of the match (driver presses STOP)
        double armPower = 1.0;
        armRotator.setTargetPosition(0);
        armRotator.setPower(armPower);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double extendableArmPower = 1.0;
        int extendableArmPosition = 0;
        extendableArm.setTargetPosition(extendableArmPosition);
        extendableArm.setDirection(DcMotor.Direction.REVERSE);
        extendableArm.setPower(extendableArmPower);
        extendableArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendableArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double driveSlower = 1.0;
        boolean slowMode = false;
        while (opModeIsActive()) {

            double max;
            double hatUp = gamepad1.dpad_up ? 1.0 : 0.0;
            double hatDown = gamepad1.dpad_down ? 1.0 : 0.0;
            double hatRight = gamepad1.dpad_right ? 1.0 : 0.0;
            double hatLeft = gamepad1.dpad_left ? 1.0 : 0.0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y + hatUp - hatDown;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x + hatRight - hatLeft;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            driveSlower = 1.0 - gamepad1.right_trigger;
            double leftFrontPower  = driveSlower*(axial + lateral + yaw);
            double rightFrontPower = driveSlower*(axial - lateral - yaw);
            double leftBackPower   = driveSlower*(axial - lateral + yaw);
            double rightBackPower  = driveSlower*(axial + lateral - yaw);

            // double armRotatorPower = 0.6*-gamepad2.left_stick_y;
            int limitChanger = 1;


            if(gamepad2.dpad_down){
                setArmExtensionTargetPositionMM(400);
                setWristTargetPositionDegrees(85);
                setArmTargetPositionDegrees(-20);
            } else if (gamepad2.dpad_up) {

                setArmExtensionTargetPositionMM(675);
                setArmTargetPositionDegrees(85);
                setWristTargetPositionDegrees(90);

            } else if (gamepad2.dpad_left) {
                setArmExtensionTargetPositionMM(250);
                sleep(500);
                setArmTargetPositionDegrees(65);

            } else if (gamepad2.dpad_right) {
                setArmExtensionTargetPositionMM(400);
                sleep(500);
                armRotator.setTargetPosition(0);
            }

            // armRotator.setPower(armRotatorPower);

            if (gamepad2.x){
                //intakePower = 1;
                intake.setPosition(0.75);


            }
            if (gamepad2.y) {
                //intakePower = -1;
                intake.setPosition(0);
            }

            if (gamepad2.left_bumper){
                //wristreverse();
                //intakePower = 1;
                setWristTargetPositionDegrees(134);
            }

            if (gamepad2.right_bumper) {
                //wristforward();
                //intakePower = -1;
                setWristTargetPositionDegrees(0);
            }

            if(gamepad2.a){
                setArmExtensionTargetPositionMM(0);
                sleep(500);
                setWristTargetPositionDegrees(90);
                intake.setPosition(0);
                setArmTargetPositionDegrees(0);

            }
            if(gamepad2.b){
                setArmExtensionTargetPositionMM(0);
            }




            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

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



            /*

            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            //intake.setPower(intakePower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Position", "%d", armRotator.getCurrentPosition());
            telemetry.addData("Extend Position", "%d", extendableArm.getCurrentPosition());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.update();

        }
    }}
