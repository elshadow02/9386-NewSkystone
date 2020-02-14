package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    public DcMotor iL = null;
    public DcMotor iR = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        iL = hardwareMap.get(DcMotor.class, "iL");
        iR = hardwareMap.get(DcMotor.class, "iR");
        iL.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        iL.setPower(1);
        iR.setPower(1);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-42, 20, 0.5*Math.PI))
                        .build()
        );
    }
}
