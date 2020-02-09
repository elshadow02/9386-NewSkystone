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

        MintyPoncho mintyPoncho = new MintyPoncho(this);

        mintyPoncho.setMode(RobotMode.STOP);

        waitForStart();

        mintyPoncho.setMode(RobotMode.BEAST_MODE);



        while (opModeIsActive()) {
            mintyPoncho.update();

            telemetry.addData("Mode", mintyPoncho.getMode());
            telemetry.addData("Lift pos", mintyPoncho.liftPos());
            telemetry.update();

        }
    }

}

