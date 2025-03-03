package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@Autonomous(name = "BlueBasketAuto", group = "Autonomous")
public class TopHatsSampleAuto extends LinearOpMode {

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
    public class gamepadUpMM implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmExtensionTargetPositionMM(675);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class gamepadUpAngle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmTargetPositionDegrees(80);
            setWristTargetPositionDegrees(90);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class button implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmExtensionTargetPositionMM(0);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class gamepadTwoY implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPosition(0);
            return false;
        }
    }
    public class gamepadTwoX implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intake.setPosition(0.75);
            return false;
        }
    }
    public class gamepadTwoLB implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0);
            return false;
        }
    }
    public class gamepadTwoLSB implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setWristTargetPositionDegrees(0);
            setArmTargetPositionDegrees(0);

            setArmExtensionTargetPositionMM(200);
            sleep(1000);
            intake.setPosition(0.75);
            sleep(500);
            wrist.setPosition(0);
            sleep(500);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }

    public class gamepadTwoRSB implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmExtensionTargetPositionMM(175);
            sleep(500);
            setWristTargetPositionDegrees(75);
            setArmTargetPositionDegrees(70);

            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class gamepadDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmExtensionTargetPositionMM(100);
            setWristTargetPositionDegrees(60);
            setArmTargetPositionDegrees(-30);
            intake.setPosition(0);
            sleep(1000);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class wait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sleep(2000);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class gamepadA implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sleep(500);
            setArmExtensionTargetPositionMM(0);
            sleep(500);
            setWristTargetPositionDegrees(90);
            setArmTargetPositionDegrees(0);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }
    public class wait200 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sleep(200);
            return false;
        }
    }
    public class wait500 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
        sleep(500);
            return false;
        }
    }
    public class gamepadUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmExtensionTargetPositionMM(675);
            setArmTargetPositionDegrees(85);
            setWristTargetPositionDegrees(90);
            if (extendableArm.isBusy()) {
                return true;
            }
            return false;
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        armRotator = hardwareMap.get(DcMotor.class, "m4");
        extendableArm = hardwareMap.get(DcMotor.class, "m5");
        intake = hardwareMap.get(ServoImplEx.class,  "s1");
        wrist = hardwareMap.get(ServoImplEx.class,  "s2");
        armRotator.setDirection(DcMotor.Direction.REVERSE);



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


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder firstRung = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(4,26), Math.toRadians(135))
                .waitSeconds(0.5);

        TrajectoryActionBuilder secondBasket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(8,18), Math.toRadians(143));
        TrajectoryActionBuilder thirdBasket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(8,18), Math.toRadians(135));
        TrajectoryActionBuilder waitone = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionCloseOut = firstRung.endTrajectory().fresh()
                .splineTo(new Vector2d(6,22), Math.toRadians(135))
                .build();

        TrajectoryActionBuilder secondRung = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(30,13), Math.toRadians(0));
        TrajectoryActionBuilder thirdRung = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(30,17), Math.toRadians(0));
        TrajectoryActionBuilder fourthRung = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(30,17), Math.toRadians(0));
        TrajectoryActionBuilder driveNSlide = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(22,-38))
                .waitSeconds(1);


        // actions that need to happen on init; for instance, a claw tightening.

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = null;
        Action secondRungAction = null;
        Action driveNSlideAction = null;
        Action waitOneAction = null;
        Action waitTwoAction = null;
        Action secondBasketAction = null;
        Action thirdRungAction = null;
        Action finalAction = null;
        Action thirdBasketAction = null;
        if (startPosition == 1) {
            intake.setPosition(0.75);
            trajectoryActionChosen = firstRung.build();
            secondRungAction = secondRung.build();
            driveNSlideAction = driveNSlide.build();
            secondBasketAction = secondBasket.build();
            thirdRungAction = thirdRung.build();
            thirdBasketAction = thirdBasket.build();
            waitOneAction = waitone.build();
            waitTwoAction = waitone.build();
            finalAction = fourthRung.build();
            Action waitThreeAction = waitone.build();
            Action waitFourAction = waitone.build();
            Action waitFiveAction = waitone.build();
            Action waitSixAction = waitone.build();
            Action waitSevenAction = waitone.build();
            Action waitEightAction = waitone.build();
            setArmExtensionTargetPositionMM(675);
            setArmTargetPositionDegrees(85);
            setWristTargetPositionDegrees(90);
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            new gamepadTwoY(),
                            secondRungAction,
                            new gamepadA(),
                            thirdRungAction
                    )
            );
        }


    }
}