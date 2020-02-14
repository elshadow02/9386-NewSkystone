package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(group = "drive")
public class RedQuarryPathTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap, this);
        arm.setMode(MotorMode.AUTO);
        arm.setPower(0);
        arm.setMaxPower(0.55);
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
                        .splineTo(new Pose2d(-33.5, -20, 0.5*Math.PI))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(0, -39, Math.PI))
                        .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                        .addMarker(new Vector2d(49, -24), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            claw.grab();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(52, -17))
                        .build()
        );

        sleep(250);

        intake.setMode(IntakeMode.STOP);
        intake.update();

        arm.stack();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(27, -45, Math.PI))
                .addMarker(() -> {
                    claw.intake();

                    arm.intake();

                    grabber.setMode(IntakeMode.PREMATCH);
                    grabber.update();

                    intake.setMode(IntakeMode.PULL);
                    intake.update();

                    return Unit.INSTANCE;
                })
                .back(19)
                .build()
        );

        sleep(200);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(0, -39, Math.PI))
                .splineTo(new Pose2d(-39, -41, Math.PI))
                .splineTo(new Pose2d(-61, -19, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(46, -40, Math.PI))
                .addMarker(new Vector2d(7, -35), () -> {
                    claw.grab();
                    sleep(150);
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();
        sleep(150);
        arm.intake();
        sleep(250);
        intake.setMode(IntakeMode.PULL);
        intake.update();claw.intake();
        arm.intake();
        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-25, -13, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(48, -40, Math.PI))
                .addMarker(new Vector2d(0, -32), () -> {
                    claw.grab();
                    sleep(150);
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();
        sleep(250);
        arm.intake();
        sleep(250);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(40)
                .build()
        );
    }
}
