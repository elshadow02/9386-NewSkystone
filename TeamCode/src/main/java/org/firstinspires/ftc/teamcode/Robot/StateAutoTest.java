package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="StateAutoTest")
public class StateAutoTest extends LinearOpMode {

    @Override
    public void runOpMode () {

        MintyPancho mintyPoncho = new MintyPancho(this);

        mintyPoncho.setMode(RobotMode.STOP);

        waitForStart();

        mintyPoncho.setMode(RobotMode.RED_QUARRY_AUTO1);
    }

}

