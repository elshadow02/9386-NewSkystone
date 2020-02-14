package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "RedQuarryPaths")
public class BQPTest2 extends LinearOpMode {
    public DcMotor iL = null;
    public DcMotor iR = null;
    public Servo iLS = null;
    public Servo iRS = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        iL = hardwareMap.get(DcMotor.class, "iL");
        iR = hardwareMap.get(DcMotor.class, "iR");
        iL.setDirection(DcMotorSimple.Direction.REVERSE);

        iLS = hardwareMap.get(Servo.class, "iLS");
        iRS = hardwareMap.get(Servo.class, "iRS");

        drive.setPoseEstimate(new Pose2d(-13, 62.5, 1.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        iLS.setPosition(0.3);
        iRS.setPosition(1.0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-33.5, 20, 1.5*Math.PI))
                        .addMarker(new Vector2d(-20, 43), () -> {
                            iL.setPower(1);
                            iR.setPower(1);
                            return Unit.INSTANCE;
                        })
//                        .lineTo(new Vector2d(-28, -15))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, 40, Math.PI))
                        .splineTo(new Pose2d(50, 23, 0.5*Math.PI))
                        .addMarker(new Vector2d(35, 33), () -> {
//                            fL.setPosition(0);
//                            fR.setPosition(0);
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        iL.setPower(0);
        iR.setPower(0);
//        claw.setPosition(0.04);
//
//        arm.setPower(0.4);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, 45, Math.PI))
                .back(12)
                .build()
        );

        //arm.setMode(MotorMode.AUTO);
        while(drive.isBusy()){
            //arm.stack();
            drive.update();
        }

//        claw.setPosition(0.5);
//        arm.setPower(-0.3);

        //claw.setPosition(0.5);

        //arm.intake();

//        fL.setPosition(1.0);
//        fR.setPosition(1.0);

        iL.setPower(0.8);
        iR.setPower(0.8);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-52, 15, 1.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, 53, Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-21, 15, 1.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, 65, Math.PI))
                .build()
        );
    }
}
