package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public double newAngle = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //drive.setPoseEstimate(new Pose2d());

            if (isStopRequested()) return;

            drive.turnSync(Math.toRadians(ANGLE));

            sleep(2000);

            if (newAngle == 4){
                newAngle = 0;
            }
            else{
                newAngle++;
            }
        }
    }
}
