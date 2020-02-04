package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MintyPoncho {
    private RobotMode mode = RobotMode.STOP;
    private OpMode opMode = null;

    private Arm arm = null;
    private DriveTrain drive = null;
    private Intake intake = null;
    private Lift lift = null;

    public MintyPoncho(OpMode opmode){
        arm = new Arm(opmode.hardwareMap, opmode);
        lift = new Lift(opmode.hardwareMap, opmode);
        intake = new Intake(opmode.hardwareMap, opmode);
        drive = new DriveTrain(opmode.hardwareMap, opmode);
    }

    public void update(){
        switch(mode){
            case STOP:
                arm.setMode(MotorMode.STOP);
                lift.setMode(MotorMode.STOP);
                drive.setDriveMode(DriveMode.STOP);
                intake.setMode(IntakeMode.STOP);
            case TURTLE_MODE:
                drive.setDriveMode(DriveMode.TURTLE_SPEED);
            case BEAST_MODE:
                drive.setDriveMode(DriveMode.MECANUM);
        }

        arm.update();
        lift.update();
        intake.update();
        drive.stateUpdate();
    }
}
