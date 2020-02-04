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

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(24)
                .build();

        Trajectory traj = drive.trajectoryBuilder()
                .splineTo(new Pose2d(40.0, 40.0, 0)).build();

        drive.setDriveMode(DriveMode.STOP);

        drive.setPoseEstimate(new Pose2d());

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                drive.setDriveMode(DriveMode.MECANUM);
            }

            if (gamepad1.b){
                drive.setDriveMode(DriveMode.TURTLE_SPEED);
            }

            if (gamepad1.x){
                drive.setDriveMode(DriveMode.COOL_MECANUM);
            }

            if (gamepad1.y){
                drive.setDriveMode(DriveMode.TANK);
            }

            if (gamepad1.right_bumper){
                drive.setDriveMode(DriveMode.STOP);
            }

            if (gamepad1.left_bumper){
                drive.turnSync(Math.PI);
            }

            drive.stateUpdate();

            telemetry.addData("Mode", drive.getDriveMode());
            telemetry.update();

        }
    }

}

