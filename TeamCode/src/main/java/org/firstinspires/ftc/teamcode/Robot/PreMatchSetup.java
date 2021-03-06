package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Use: Setup")
public class PreMatchSetup extends LinearOpMode {

    @Override
    public void runOpMode () {

        Lift lift = new Lift(hardwareMap, this);
        lift.reset();

        waitForStart();

        lift.setMode(MotorMode.AUTO);
        lift.nextPosition();

        while(lift.isBusy()){
            telemetry.addLine("New line");
            telemetry.update();
        }

    }

}

