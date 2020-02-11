package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="StateTest")
public class StateTest extends LinearOpMode {

    @Override
    public void runOpMode () {

        MintyPancho mintyPoncho = new MintyPancho(this);

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

