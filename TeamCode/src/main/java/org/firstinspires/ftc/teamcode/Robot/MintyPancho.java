package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

public class MintyPancho {
    private RobotMode mode = RobotMode.STOP;
    private LinearOpMode lopMode = null;

    private Arm arm = null;
    private DriveTrain drive = null;
    private Intake intake = null;
    private Lift lift = null;
    private Claw claw = null;
    private FoundationGrabber grabber= null;

    public MintyPancho(LinearOpMode opmode){
        arm = new Arm(opmode.hardwareMap, opmode);
        lift = new Lift(opmode.hardwareMap, opmode);
        intake = new Intake(opmode.hardwareMap, opmode);
        drive = new DriveTrain(opmode.hardwareMap, opmode);
        claw = new Claw(opmode);
        grabber = new FoundationGrabber(opmode.hardwareMap, opmode);

        lopMode = opmode;
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
            case RED_QUARRY_AUTO3:
                redQuarryPath3();
                break;
            case RED_FOUNDATION:
                redFoundation();
                break;
            case BLUE_FOUNDATION:
                blueFoundation();
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

    private void redFoundation() {
        while (lopMode.opModeIsActive()){
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5 * Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(50.0, -25.0, 1.5 * Math.PI))
                            .addMarker(() -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                lopMode.sleep(250);
                                return Unit.INSTANCE;
                            })
                            .reverse().splineTo(new Pose2d(27.0, -45.0, Math.PI))
                            .addMarker(() -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                return Unit.INSTANCE;
                            })
                            .back(16.0)
                            .splineTo(new Pose2d(0.0, -62.0, Math.PI))
                            .build()
            );

            break;
        }
    }

    private void blueFoundation() {
        while (lopMode.opModeIsActive()){
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, 62.5, 0.5*Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(50.0, 25.0, 0.5*Math.PI))
                            .addMarker(() -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                lopMode.sleep(250);
                                return Unit.INSTANCE;
                            })
                            .reverse().splineTo(new Pose2d(27.0, 45.0, Math.PI))
                            .addMarker(() -> {
                                grabber.setMode(IntakeMode.PREMATCH);
                                grabber.update();
                                return Unit.INSTANCE;
                            })
                            .back(16.0)
                            .splineTo(new Pose2d(0.0, 62.0, Math.PI))
                            .build()
            );

            break;
        }
    }

    private void redQuarryPath1(){
        while(lopMode.opModeIsActive()) {
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-25, -22, 0.5*Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -46, Math.PI))
                            .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                            .addMarker(new Vector2d(49, -24), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(52, -17))
                            .build()
            );

            lopMode.sleep(250);

            intake.setMode(IntakeMode.STOP);
            intake.update();
            claw.grab();

            drive.followTrajectory(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(27, -45, Math.PI))
                    .back(12)
                    .build()
            );

            arm.stack();

            while(drive.isBusy()){
                arm.stack();
                drive.update();
            }



            claw.intake();

            lopMode.sleep(150);

            arm.intake();

            grabber.setMode(IntakeMode.PREMATCH);
            grabber.update();

            intake.setMode(IntakeMode.PULL);
            intake.update();

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
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();claw.intake();
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
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(20)
                    .build()
            );

            break;
        }
    }

    private void redQuarryPath2(){
        while(lopMode.opModeIsActive()) {
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5 * Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-33.5, -20, 0.5 * Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                            .splineTo(new Pose2d(45, -23, 1.5 * Math.PI))
                            .addMarker(new Vector2d(40, -28), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                claw.grab();
                                return Unit.INSTANCE;
                            })
                            .build()
            );

            intake.setMode(IntakeMode.STOP);
            intake.update();

            drive.followTrajectory(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(35, -45, Math.PI))
                    .back(12)
                    .build()
            );

            while (drive.isBusy()) {
                arm.stack();
                drive.update();
            }

            claw.intake();

            lopMode.sleep(150);

            arm.intake();

            grabber.setMode(IntakeMode.PREMATCH);
            grabber.update();

            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-52, -15, 0.5 * Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(45, -53, Math.PI))
                    .addMarker(new Vector2d(0, -30), () -> {
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        claw.grab();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-21, -15, 0.5 * Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(45, -65, Math.PI))
                    .addMarker(new Vector2d(0, -35), () -> {
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        claw.grab();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(20)
                    .build()
            );

            break;
        }
    }

    private void redQuarryPath3(){
        while(lopMode.opModeIsActive()) {
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5 * Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-41, -20, 0.5*Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -40, Math.PI))
                            .splineTo(new Pose2d(45, -23, 1.5*Math.PI))
                            .addMarker(new Vector2d(40, -28), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                claw.grab();
                                return Unit.INSTANCE;
                            })
                            .build()
            );

            intake.setMode(IntakeMode.STOP);
            intake.update();

            drive.followTrajectory(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(35, -45, Math.PI))
                    .back(12)
                    .build()
            );

            while(drive.isBusy()){
                arm.stack();
                drive.update();
            }

            claw.intake();

            lopMode.sleep(150);

            arm.intake();

            grabber.setMode(IntakeMode.PREMATCH);
            grabber.update();

            intake.setMode(IntakeMode.PULL);
            intake.update();


            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-71, -16, 0.75*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(45, -39, Math.PI))
                    .addMarker(new Vector2d(0, -30), () -> {
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        claw.grab();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-26, -7, 0.5*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(45, -49, Math.PI))
                    .addMarker(new Vector2d(0, -30), () -> {
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        claw.grab();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(250);
            arm.intake();
            lopMode.sleep(250);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(20)
                    .build()
            );

            break;
        }
    }
}