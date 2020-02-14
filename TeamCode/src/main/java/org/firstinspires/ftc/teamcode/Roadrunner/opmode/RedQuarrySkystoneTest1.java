package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.ClawMode;
import org.firstinspires.ftc.teamcode.Robot.FoundationGrabber;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeMode;
import org.firstinspires.ftc.teamcode.Robot.MotorMode;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class RedQuarrySkystoneTest1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap, this);
        arm.setMode(MotorMode.AUTO);
        arm.setPower(0);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Intake intake = new Intake(hardwareMap, this);
        intake.setMaxPower(1.0);
        Claw claw = new Claw(this);
        claw.setMode(ClawMode.AUTO);
        FoundationGrabber grabber = new FoundationGrabber(hardwareMap, this);
        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        claw.intake();
        intake.setMode(IntakeMode.OUT);
        intake.update();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25.0, -22.0, 0.5*Math.PI))
                        .back(25.0)
                        .splineTo(new Pose2d(16.0, -35.0, 0.0))
                        .addMarker(new Vector2d(0, -32), () -> {
                            intake.setMode(IntakeMode.PUSH);
                            intake.update();
                            sleep(250);
                            return Unit.INSTANCE;
                        })
                        .reverse().splineTo(new Pose2d(-49.0, -52.0, 0.5*Math.PI))
                        .addMarker(() -> {
                            intake.setMode(IntakeMode.PULL);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(-49.0, -22.0))
                        .back(30.0)
                        .reverse().splineTo(new Pose2d(16.0, -35.0, 0.0))
                        .reverse().splineTo(new Pose2d(-33.0, -52.0, 0.5*Math.PI))
                        .addMarker(() -> {
                            intake.setMode(IntakeMode.PULL);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(-33.0, -22.0))
                        .back(30.0)
                        .reverse().splineTo(new Pose2d(16.0, -35.0, 0.0))
                        .reverse().splineTo(new Pose2d(-41.0, -52.0, 0.5*Math.PI))
                        .addMarker(() -> {
                            intake.setMode(IntakeMode.PULL);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(-41.0, -22.0))
                        .back(30.0)
                        .reverse().splineTo(new Pose2d(16.0, -35.0, 0.0))
                        .reverse().splineTo(new Pose2d(-57.0, -52.0, 0.5*Math.PI))
                        .addMarker(() -> {
                            intake.setMode(IntakeMode.PULL);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(-57.0, -22.0))
                        .back(30.0)
                        .reverse().splineTo(new Pose2d(16.0, -35.0, 0.0))
                        .back(12.0)
                .build()
        );

    }
}
