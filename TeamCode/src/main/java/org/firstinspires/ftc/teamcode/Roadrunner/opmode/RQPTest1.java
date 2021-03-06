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
public class RQPTest1 extends LinearOpMode {

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
                .reverse().splineTo(new Pose2d(0, -46, Math.PI))
                        .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                        .addMarker(new Vector2d(45, -24), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(52, -17))
                        .build()
        );

        sleep(750);

        intake.setMode(IntakeMode.STOP);
        intake.update();
        claw.grab();


        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(27, -45, Math.PI))
                .back(12)
                .build()
        );

        arm.stack();

        while(drive.isBusy()){
            arm.stack();
            drive.update();
        }



        claw.intake();

        sleep(750);

        claw.grab();
        arm.intake();

        //arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);
        grabber.update();

        intake.setMode(IntakeMode.PULL);
        intake.update();

        claw.intake();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-41, -15, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
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
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
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
    }
}
