package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.ClawMode;
import org.firstinspires.ftc.teamcode.Robot.FoundationGrabber;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeMode;
import org.firstinspires.ftc.teamcode.Robot.MotorMode;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "RedQuarryPaths")
public class RedQuarryPathsTest extends LinearOpMode {

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
                        .splineTo(new Pose2d(-33.5, -20, 0.5*Math.PI))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                        .splineTo(new Pose2d(45, -23, 1.5*Math.PI))
                        .addMarker(new Vector2d(40, -28), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            claw.grab();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        intake.setMode(IntakeMode.STOP);
        intake.update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, -45, Math.PI))
                .back(12)
                .build()
        );

        while(drive.isBusy()){
            arm.stack();
            drive.update();
        }

        claw.intake();

        sleep(150);

        arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);
        grabber.update();

        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-52, -15, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(45, -53, Math.PI))
                .addMarker(new Vector2d(0, -30), () -> {
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    claw.grab();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();
        sleep(250);
        arm.intake();
        sleep(250);
        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-21, -15, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(45, -65, Math.PI))
                .addMarker(new Vector2d(0, -35), () -> {
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    claw.grab();
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
                .forward(20)
                .build()
        );
    }
}
