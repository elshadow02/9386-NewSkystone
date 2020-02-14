package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class RedFoundationTest extends LinearOpMode {

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
        drive.setPoseEstimate(new Pose2d(-13, -62.5, 1.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        claw.intake();
        intake.setMode(IntakeMode.OUT);
        intake.update();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(50.0, -25.0, 1.5*Math.PI))
                        .reverse().splineTo(new Pose2d(27.0, -45.0, Math.PI))
                        .back(16.0)
                        .splineTo(new Pose2d(0.0, -62.0, Math.PI))
                .build()
        );

    }
}
