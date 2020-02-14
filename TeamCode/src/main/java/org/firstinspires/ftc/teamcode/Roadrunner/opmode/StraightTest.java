package org.firstinspires.ftc.teamcode.Roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        Trajectory traj = drive.trajectoryBuilder()
                .back(DISTANCE)
                .build();

        drive.setPoseEstimate(new Pose2d(0, 0,0));

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) return;

            drive.followTrajectorySync(trajectory);

            sleep(1000);

            drive.followTrajectorySync(traj);
            sleep (1000);
        }
    }
}
