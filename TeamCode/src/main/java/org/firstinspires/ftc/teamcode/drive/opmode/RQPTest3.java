package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RQPTest3 extends LinearOpMode {
    public DcMotor iL = null;
    public DcMotor iR = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        iL = hardwareMap.get(DcMotor.class, "iL");
        iR = hardwareMap.get(DcMotor.class, "iR");
        iL.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-41, -20, 0.5*Math.PI))
                        .addMarker(new Vector2d(-20, -43), () -> {
                            iL.setPower(1);
                            iR.setPower(1);
                            return Unit.INSTANCE;
                        })
//                        .lineTo(new Vector2d(-28, -15))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                        .splineTo(new Pose2d(50, -23, 1.5*Math.PI))
                        .addMarker(new Vector2d(35, -33), () -> {
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
                .splineTo(new Pose2d(35, -45, Math.PI))
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
                .splineTo(new Pose2d(-71, -16, 0.75*Math.PI))
                .reverse().splineTo(new Pose2d(45, -39, Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-26, -7, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -49, Math.PI))
                .build()
        );
    }
}
