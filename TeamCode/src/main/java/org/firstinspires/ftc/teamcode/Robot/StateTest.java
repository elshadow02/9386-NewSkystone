package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.EEHardware;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;


@TeleOp(name="Old-TeleOp")
public class StateTest extends LinearOpMode {

    @Override
    public void runOpMode () {

        DriveTrain drive = new DriveTrain(hardwareMap, gamepad1);

        telemetry.addLine("Init Complete");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                drive.setDriveMode(DriveMode.MECANUM);
            }
            if (gamepad1.b){
                drive.setDriveMode(DriveMode.TANK);
            }
            if (gamepad1.x){
                drive.setDriveMode(DriveMode.COOL_MECANUM);
            }
            if (gamepad1.y){
                drive.setDriveMode(DriveMode.TURTLE_SPEED);
            }
            if (gamepad1.right_bumper){
                drive.setDriveMode(DriveMode.STOP);
            }
            if (gamepad1.left_bumper){
                drive.turnSync(90);
            }


            telemetry.update();

        }
    }

}

