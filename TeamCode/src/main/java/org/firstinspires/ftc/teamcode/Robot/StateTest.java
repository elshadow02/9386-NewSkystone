package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.EEHardware;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;


@TeleOp(name="StateTest")
public class StateTest extends LinearOpMode {

    @Override
    public void runOpMode () {

        DriveTrain drive = new DriveTrain(hardwareMap, this);

        telemetry.addLine("Init Complete");
        telemetry.update();

        drive.setPoseEstimate(new Pose2d());

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(24)
                .build();

        Trajectory traj = drive.trajectoryBuilder()
                .splineTo(new Pose2d(40.0, 40.0, 0)).build();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper){
                drive.turnSync(Math.toRadians(90));
            }

            if(gamepad1.a){
                drive.followTrajectorySync(trajectory);
            }

            if (gamepad1.right_bumper){
                drive.followTrajectorySync(traj);
            }

        }
    }

}

