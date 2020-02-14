package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MintyPancho;
import org.firstinspires.ftc.teamcode.Robot.RobotMode;


@TeleOp(name="RedTeleOp")
public class RedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode () {

        MintyPancho mintyPoncho = new MintyPancho(this);

        mintyPoncho.setMode(RobotMode.STOP);

        mintyPoncho.setRedSide(true);

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

