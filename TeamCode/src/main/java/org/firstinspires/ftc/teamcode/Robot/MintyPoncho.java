package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

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

    public int liftPos(){
        return lift.getLiftPosition();
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
                break;
            case TURTLE_MODE:
                drive.setDriveMode(DriveMode.TURTLE_SPEED);
                arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.INTAKE);
                grabber.setMode(IntakeMode.TELEOP);
                break;
            case BEAST_MODE:
                arm.setMaxPower(0.5);
                drive.setDriveMode(DriveMode.MECANUM);
                //arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                //lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.TELEOP);
                grabber.setMode(IntakeMode.TELEOP);
                break;
            case RED_QUARRY_AUTO1:
                redQuarryPath1();
                break;
            case RED_QUARRYAUTO2:
                redQuarryPath2();
                break;
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
        arm.setMode(MotorMode.AUTO);
        arm.setPower(0);
        intake.setMaxPower(1.0);
        claw.setMode(ClawMode.AUTO);
        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        claw.intake();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25, -22, 0.5*Math.PI))
                        .addMarker(new Vector2d(-18, -43), () -> {
                            intake.setMode(IntakeMode.OUT);
                            intake.update();
                            return Unit.INSTANCE;
                        })
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(0, -46, Math.PI))
                        .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                        .addMarker(new Vector2d(45, -24), () -> {
                            grabber.setMode(IntakeMode.OUT);
                            grabber.update();
                            return Unit.INSTANCE;
                        })
                        .lineTo(new Vector2d(52, -17))
                        .build()
        );

        //sleep(750);

        intake.setMode(IntakeMode.STOP);
        intake.update();
        claw.grab();


//        drive.followTrajectory(drive.trajectoryBuilder()
//                .splineTo(new Pose2d(27, -45, Math.PI))
//                .back(12)
//                .build()
//        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(16)
                .build()
        );

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(27, -45, Math.PI))
                .back(22)
                .build()
        );

        arm.stack();

        while(drive.isBusy()){
            arm.stack();
            drive.update();
        }



        claw.intake();

        claw.grab();
        arm.intake();

        //arm.intake();

        grabber.setMode(IntakeMode.PREMATCH);
        grabber.update();

        intake.setMode(IntakeMode.PULL);
        intake.update();

        claw.intake();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-41, -15, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(45, -35, Math.PI))
                .addMarker(new Vector2d(0, -32), () -> {
                    claw.grab();
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();
        arm.intake();
        intake.setMode(IntakeMode.PULL);
        intake.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-28, -13, 0.5*Math.PI))
                .build()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .reverse().splineTo(new Pose2d(45, -35, Math.PI))
                .addMarker(new Vector2d(0, -32), () -> {
                    claw.grab();
                    intake.setMode(IntakeMode.STOP);
                    intake.update();
                    arm.stack();
                    return Unit.INSTANCE;
                })
                .build()
        );

        claw.intake();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(20)
                .build());
    }

    private void redQuarryPath2(){
        arm.setMode(MotorMode.AUTO);
        arm.setPower(0);
        intake.setMaxPower(1.0);
        claw.setMode(ClawMode.AUTO);
        drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

        claw.intake();

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
