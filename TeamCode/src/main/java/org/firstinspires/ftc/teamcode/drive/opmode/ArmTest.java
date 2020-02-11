package org.firstinspires.ftc.teamcode.drive.opmode;

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
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap, this);
        arm.setMode(MotorMode.AUTO);
        arm.setPower(0);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Intake intake = new Intake(hardwareMap, this);
        intake.setMaxPower(0.8);
        Claw claw = new Claw(this);
        claw.setMode(ClawMode.AUTO);
        FoundationGrabber grabber = new FoundationGrabber(hardwareMap, this);
        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        arm.stack();

        sleep(3000);

        while(arm.isBusy()){
            arm.intake();
        }

        sleep(3000);

        while(arm.isBusy()){
            arm.stack();
        }

        sleep(3000);

        arm.intake();

        sleep(10000);

        claw.intake();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25, -22, 0.5*Math.PI))
                        .addMarker(new Vector2d(-18, -43), () -> {
                            intake.setMode(IntakeMode.OUT);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(0, -42, Math.PI))
                        .splineTo(new Pose2d(45, -23, 1.5*Math.PI))
                        .addMarker(new Vector2d(40, -33), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        intake.setMode(IntakeMode.STOP);
        intake.update();
        claw.grab();


        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, -35, Math.PI))
                .addMarker(new Vector2d(35, -45), () -> {
                    claw.intake();
                    return Unit.INSTANCE;
                })
                .back(12)
                .build()
        );

        while(drive.isBusy()){
            arm.stack();
            drive.update();
        }

        claw.grab();
        arm.intake();

        //arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);
        grabber.update();

        intake.setMode(IntakeMode.PULL);
        intake.update();

        claw.intake();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-52, -15, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -35, Math.PI))
                .addMarker(new Vector2d(0, -32), () -> {
                    claw.grab();
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();
        arm.intake();
        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-28, -13, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -35, Math.PI))
                .addMarker(new Vector2d(0, -32), () -> {
                    claw.grab();
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(20)
                .build());


        //sleep(2000);

//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(30, 30, 0))
//                        .build()
//        );
//
//        sleep(2000);
//
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .reverse()
//                .splineTo(new Pose2d(0, 0, 0))
//                .build()
//        );
    }
}
