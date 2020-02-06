package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import kotlin.Unit;

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

    private void blueQuarryPath1(){

    }

    private void redQuarryPath1(){
        claw.setMode(ClawMode.AUTO);
        arm.setMode(MotorMode.AUTO);
        lift.setMode(MotorMode.AUTO);

        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25, -23, 0.5*Math.PI))
                        .addMarker(new Vector2d(-18, -43), () -> {
                            intake.setMode(IntakeMode.OUT);
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                        .splineTo(new Pose2d(45, -23, 1.5*Math.PI))
                        .addMarker(new Vector2d(35, -33), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        intake.setMode(IntakeMode.STOP);
        intake.update();

        //claw.grab();

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, -45, Math.PI))
                .back(12)
                .build()
        );

        while(drive.isBusy()){
            //arm.stack();
            drive.update();
        }

//        claw.intake();

        //arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);

        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-49.5, -15, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -53, Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-28, -15, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -65, Math.PI))
                .build()
        );
    }

    private void redQuarryPath2(){
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-33.5, -20, 0.5*Math.PI))
                        .addMarker(new Vector2d(-20, -43), () -> {
                            intake.setMode(IntakeMode.OUT);
                            intake.update();
                            return Unit.INSTANCE;
                        })
//                        .lineTo(new Vector2d(-28, -15))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                        .splineTo(new Pose2d(45, -23, 1.5*Math.PI))
                        .addMarker(new Vector2d(35, -33), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        intake.setMode(IntakeMode.STOP);
        intake.update();
//        claw.grab();

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, -45, Math.PI))
                .back(12)
                .build()
        );

        while(drive.isBusy()){
            //arm.stack();
            drive.update();
        }

//        claw.intake();

        //arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);
        grabber.update();

        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-52, -15, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -53, Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-21, -15, 0.5*Math.PI))
                .reverse().splineTo(new Pose2d(45, -65, Math.PI))
                .build()
        );
    }
}
