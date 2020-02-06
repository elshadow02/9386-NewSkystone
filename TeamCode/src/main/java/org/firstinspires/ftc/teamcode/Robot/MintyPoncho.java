package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MintyPoncho {
    private RobotMode mode = RobotMode.STOP;
    private OpMode opMode = null;

    private Arm arm = null;
    private DriveTrain drive = null;
    private Intake intake = null;
    private Lift lift = null;
    private Claw claw = null;
    private FoundationGrabber grabber= null;

    public MintyPoncho(OpMode opmode){
        arm = new Arm(opmode.hardwareMap, opmode);
        lift = new Lift(opmode.hardwareMap, opmode);
        intake = new Intake(opmode.hardwareMap, opmode);
        drive = new DriveTrain(opmode.hardwareMap, opmode);
        claw = new Claw(opmode);
        grabber = new FoundationGrabber(opmode.hardwareMap, opmode);
    }

    public void setMode(RobotMode newMode){
        this.mode = newMode;
    }

    public RobotMode getMode(){
        return this.mode;
    }

    public void update(){
        switch(mode){
            case STOP:
                arm.setMode(MotorMode.STOP);
                lift.setMode(MotorMode.STOP);
                drive.setDriveMode(DriveMode.STOP);
                intake.setMode(IntakeMode.STOP);
                claw.setMode(ClawMode.STOP);
                grabber.setMode(IntakeMode.STOP);
            case TURTLE_MODE:
                drive.setDriveMode(DriveMode.TURTLE_SPEED);
                arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.INTAKE);
                grabber.setMode(IntakeMode.TELEOP);
            case BEAST_MODE:
                drive.setDriveMode(DriveMode.MECANUM);
                arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.INTAKE);
                grabber.setMode(IntakeMode.TELEOP);
        }

        arm.update();
        lift.update();
        intake.update();
        drive.stateUpdate();
        claw.update();
        grabber.update();
    }
}
