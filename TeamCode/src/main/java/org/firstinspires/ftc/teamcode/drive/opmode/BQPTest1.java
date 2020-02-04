package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BQPTest1 extends LinearOpMode {
    public DcMotor iL = null;
    public DcMotor iR = null;
    public Servo iLS = null;
    public Servo iRS = null;
    public Servo fL = null;
    public Servo fR = null;
    public Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap, this);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        iL = hardwareMap.get(DcMotor.class, "iL");
        iR = hardwareMap.get(DcMotor.class, "iR");
        iL.setDirection(DcMotorSimple.Direction.REVERSE);

        iLS = hardwareMap.get(Servo.class, "iLS");
        iRS = hardwareMap.get(Servo.class, "iRS");
        fL = hardwareMap.get(Servo.class, "foundL");
        fR = hardwareMap.get(Servo.class, "foundR");
        claw = hardwareMap.get(Servo.class, "claw");

        drive.setPoseEstimate(new Pose2d(-13, 62.5, 1.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25, 23, 1.5*Math.PI))
                        .addMarker(new Vector2d(-18, 43), () -> {
                            iL.setPower(0.8);
                            iR.setPower(0.8);
                            return Unit.INSTANCE;
                        })
                        .build()
        );

//        iL.setPower(0);
//        iR.setPower(0);
        //claw.setPosition(0.04);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(0, 40, Math.PI))
                        .splineTo(new Pose2d(59, 23, 0.5*Math.PI))
                        .addMarker(new Vector2d(35, 33), () -> {
                            fL.setPosition(0);
                            fR.setPosition(0);
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

        fL.setPosition(1.0);
        fR.setPosition(1.0);

        iL.setPower(0.8);
        iR.setPower(0.8);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-49.5, 15, 1.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, 53, Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-28, -15, 1.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, 65, Math.PI))
                .build()
        );
    }
}
