package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Servo cap = null;

    private boolean changed = false;
    private boolean newChanged = false;
    private boolean redSide = false;

    public MintyPancho(LinearOpMode opmode){
        arm = new Arm(opmode.hardwareMap, opmode);
        lift = new Lift(opmode.hardwareMap, opmode);
        intake = new Intake(opmode.hardwareMap, opmode);
        drive = new DriveTrain(opmode.hardwareMap, opmode);
        claw = new Claw(opmode);
        grabber = new FoundationGrabber(opmode.hardwareMap, opmode);

        cap = opmode.hardwareMap.get(Servo.class, "cap");

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

    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }

    public void update(){
        if(!changed && lopMode.gamepad1.y){
            if(mode == RobotMode.BEAST_MODE){
                setMode(RobotMode.TURTLE_MODE);
            }
            else if(mode == RobotMode.TURTLE_MODE){
                setMode(RobotMode.BEAST_MODE);
            }

            changed = true;
        }
        else if(!lopMode.gamepad1.y && changed){
            changed = false;
        }
        if(mode == RobotMode.TURTLE_MODE || mode == RobotMode.BEAST_MODE){
            if(lopMode.gamepad2.y){
                cap.setPosition(0.9);
            }
            if(lopMode.gamepad2.a){
                cap.setPosition(0.1);
            }
        }
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
                //arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                //lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.TELEOP);
                grabber.setMode(IntakeMode.TELEOP);

                if(!newChanged && redSide){
                    arm.setMode(MotorMode.CONTROLLED);
                    lift.setMode(MotorMode.CONTROLLED);
                    newChanged = true;
                }

                if(!newChanged && !redSide){
                    arm.setMode(MotorMode.CONTROLLED);
                    lift.setMode(MotorMode.CONTROLLED);
                    newChanged = true;
                }
                break;
            case BEAST_MODE:
                arm.setMaxPower(0.55);
                lift.setMaxPower(0.8);
                drive.setDriveMode(DriveMode.MECANUM);
                //arm.setMode(MotorMode.CONTROLLED);
                intake.setMode(IntakeMode.TELEOP);
                //lift.setMode(MotorMode.CONTROLLED);
                claw.setMode(ClawMode.TELEOP);
                grabber.setMode(IntakeMode.TELEOP);

                if(!newChanged && redSide){
                    arm.setMode(MotorMode.CONTROLLED);
                    lift.setMode(MotorMode.CONTROLLED);
                    arm.setRedSide(true);
                    newChanged = true;
                }

                if(!newChanged && !redSide){
                    arm.setMode(MotorMode.CONTROLLED);
                    lift.setMode(MotorMode.CONTROLLED);
                    arm.setRedSide(false);
                    newChanged = true;
                }
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
            case BLUE_QUARRY_AUTO1:
                blueQuarryPath1();
                break;
            case BLUE_QUARRY_AUTO2:
                blueQuarryPath2();
                break;
            case BLUE_QUARRY_AUTO3:
                blueQuarryPath3();
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
            lift.setMode(MotorMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5*Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();
            lift.downPosition();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-25, -22, 0.5*Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -39, Math.PI))
                            .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                            .addMarker(new Vector2d(49, -24), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                claw.grab();
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(52, -17))
                            .build()
            );

            lopMode.sleep(250);

            intake.setMode(IntakeMode.STOP);
            intake.update();

            arm.stack();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(27, -45, Math.PI))
                    .addMarker(() -> {
                        lopMode.sleep(100);
                        claw.intake();

                        arm.intake();

                        grabber.setMode(IntakeMode.PREMATCH);
                        grabber.update();

                        intake.setMaxPower(0.6);
                        intake.setMode(IntakeMode.PULL);
                        intake.update();

                        return Unit.INSTANCE;
                    })
                    .back(19)
                    .build()
            );

            lopMode.sleep(150);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, -39, Math.PI))
                    .splineTo(new Pose2d(-39, -41, Math.PI))
                    .splineTo(new Pose2d(-50, -19, 0.5*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(46, -40, Math.PI))
                    .addMarker(new Vector2d(7, -35), () -> {
                        claw.grab();
                        lopMode.sleep(150);
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            lopMode.sleep(200);

            claw.intake();
            lopMode.sleep(150);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();claw.intake();
            arm.intake();
            intake.setMaxPower(0.6);
            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-31, -10, 0.5*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(48, -40, Math.PI))
                    .addMarker(new Vector2d(0, -32), () -> {
                        claw.grab();
                        lopMode.sleep(150);
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
                    .forward(40)
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
            lift.setMode(MotorMode.AUTO);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5 * Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();
            lift.downPosition();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-33.5, -20, 0.5*Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -39, Math.PI))
                            .splineTo(new Pose2d(52, -22, 1.5*Math.PI))
                            .addMarker(new Vector2d(49, -24), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                claw.grab();
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(52, -17))
                            .build()
            );

            lopMode.sleep(250);

            intake.setMode(IntakeMode.STOP);
            intake.update();

            arm.stack();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(27, -45, Math.PI))
                    .addMarker(() -> {
                        claw.intake();

                        arm.intake();

                        grabber.setMode(IntakeMode.PREMATCH);
                        grabber.update();

                        intake.setMode(IntakeMode.PULL);
                        intake.update();

                        return Unit.INSTANCE;
                    })
                    .back(19)
                    .build()
            );

            lopMode.sleep(200);

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, -39, Math.PI))
                    .splineTo(new Pose2d(-39, -41, Math.PI))
                    .splineTo(new Pose2d(-61, -19, 0.5*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(46, -40, Math.PI))
                    .addMarker(new Vector2d(7, -35), () -> {
                        claw.grab();
                        lopMode.sleep(150);
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(150);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();claw.intake();
            arm.intake();
            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-25, -13, 0.5*Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(48, -40, Math.PI))
                    .addMarker(new Vector2d(0, -32), () -> {
                        claw.grab();
                        lopMode.sleep(150);
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
                    .forward(40)
                    .build()
            );

            break;
        }
    }

    private void redQuarryPath3() {
        while (lopMode.opModeIsActive()) {
            arm.setMode(MotorMode.AUTO);
            arm.setPower(0);
            intake.setMaxPower(1.0);
            lift.setMode(MotorMode.AUTO);
            claw.setMode(ClawMode.AUTO);
            drive.setPoseEstimate(new Pose2d(-13, -62.5, 0.5 * Math.PI));

            claw.intake();
            intake.setMode(IntakeMode.OUT);
            intake.update();
            lift.downPosition();

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-41, -20, 0.5 * Math.PI))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse().splineTo(new Pose2d(0, -39, Math.PI))
                            .splineTo(new Pose2d(52, -22, 1.5 * Math.PI))
                            .addMarker(new Vector2d(49, -24), () -> {
                                grabber.setMode(IntakeMode.OUT);
                                grabber.update();
                                claw.grab();
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(52, -17))
                            .build()
            );

            lopMode.sleep(250);

            intake.setMode(IntakeMode.STOP);
            intake.update();

            arm.stack();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(27, -45, Math.PI))
                    .addMarker(() -> {
                        claw.intake();

                        arm.intake();

                        grabber.setMode(IntakeMode.PREMATCH);
                        grabber.update();

                        intake.setMode(IntakeMode.PULL);
                        intake.update();

                        return Unit.INSTANCE;
                    })
                    .back(19)
                    .build()
            );

            lopMode.sleep(200);

//        drive.followTrajectorySync(drive.trajectoryBuilder()
//                .splineTo(new Pose2d(0, -39, Math.PI))
//                .splineTo(new Pose2d(-66, -19, 0.8*Math.PI))
//                .build()
//        );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, -39, Math.PI))
                    .splineTo(new Pose2d(-38, -39, Math.PI))
                    .strafeRight(35)
                    .forward(20)
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .back(12)
                    .strafeLeft(45)
                    .reverse().splineTo(new Pose2d(49, -45, Math.PI))
                    .addMarker(new Vector2d(7, -35), () -> {
                        claw.grab();
                        lopMode.sleep(150);
                        intake.setMode(IntakeMode.STOP);
                        intake.update();
                        arm.stack();
                        return Unit.INSTANCE;
                    })
                    .build()
            );

            claw.intake();
            lopMode.sleep(150);
            arm.intake();
            lopMode.sleep(250);
            intake.setMode(IntakeMode.PULL);
            intake.update();
            claw.intake();
            arm.intake();
            intake.setMode(IntakeMode.PULL);
            intake.update();

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-31, -13, 0.5 * Math.PI))
                    .build()
            );

            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .reverse().splineTo(new Pose2d(48, -40, Math.PI))
                    .addMarker(new Vector2d(0, -32), () -> {
                        claw.grab();
                        lopMode.sleep(150);
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
                    .forward(40)
                    .build()
            );

            break;
        }
    }

        private void blueQuarryPath1(){
            while(lopMode.opModeIsActive()) {
                arm.setMode(MotorMode.AUTO);
                arm.setPower(0);
                intake.setMaxPower(1.0);
                lift.setMode(MotorMode.AUTO);
                claw.setMode(ClawMode.AUTO);
                drive.setPoseEstimate(new Pose2d(-13, 62.5, 1.5 * Math.PI));

                claw.intake();
                intake.setMode(IntakeMode.OUT);
                intake.update();
                lift.downPosition();

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-25, 22, 1.5*Math.PI))
                                .build()
                );

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse().splineTo(new Pose2d(0, 39, Math.PI))
                                .splineTo(new Pose2d(52, 22, 0.5*Math.PI))
                                .addMarker(new Vector2d(49, 24), () -> {
                                    grabber.setMode(IntakeMode.OUT);
                                    grabber.update();
                                    claw.grab();
                                    return Unit.INSTANCE;
                                })
                                .lineTo(new Vector2d(52, 17))
                                .build()
                );

                lopMode.sleep(250);

                intake.setMode(IntakeMode.STOP);
                intake.update();

                arm.stack();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(27, 45, Math.PI))
                        .addMarker(() -> {
                            claw.intake();

                            arm.intake();

                            grabber.setMode(IntakeMode.PREMATCH);
                            grabber.update();

                            intake.setMode(IntakeMode.PULL);
                            intake.update();

                            return Unit.INSTANCE;
                        })
                        .back(19)
                        .build()
                );

                lopMode.sleep(200);

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 39, Math.PI))
                        .splineTo(new Pose2d(-39, 41, Math.PI))
                        .splineTo(new Pose2d(-50, 19, 1.5*Math.PI))
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(46, 40, Math.PI))
                        .addMarker(new Vector2d(7, 35), () -> {
                            claw.grab();
                            lopMode.sleep(150);
                            intake.setMode(IntakeMode.STOP);
                            intake.update();
                            arm.stack();
                            return Unit.INSTANCE;
                        })
                        .build()
                );

                claw.intake();
                lopMode.sleep(150);
                arm.intake();
                lopMode.sleep(250);
                intake.setMode(IntakeMode.PULL);
                intake.update();claw.intake();
                arm.intake();
                intake.setMode(IntakeMode.PULL);
                intake.update();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-31, 10, 1.5*Math.PI))
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(48, 40, Math.PI))
                        .addMarker(new Vector2d(0, 32), () -> {
                            claw.grab();
                            lopMode.sleep(150);
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
                        .forward(40)
                        .build()
                );
            }
        }

        private void blueQuarryPath2(){
            while(lopMode.opModeIsActive()) {
                arm.setMode(MotorMode.AUTO);
                arm.setPower(0);
                intake.setMaxPower(1.0);
                lift.setMode(MotorMode.AUTO);
                claw.setMode(ClawMode.AUTO);
                drive.setPoseEstimate(new Pose2d(-13, 62.5, 1.5 * Math.PI));

                claw.intake();
                intake.setMode(IntakeMode.OUT);
                intake.update();
                lift.downPosition();

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-35, 20, 1.5*Math.PI))
                                .build()
                );

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse().splineTo(new Pose2d(0, 39, Math.PI))
                                .splineTo(new Pose2d(52, 22, 0.5*Math.PI))
                                .addMarker(new Vector2d(49, 24), () -> {
                                    grabber.setMode(IntakeMode.OUT);
                                    grabber.update();
                                    claw.grab();
                                    return Unit.INSTANCE;
                                })
                                .lineTo(new Vector2d(52, 17))
                                .build()
                );

                lopMode.sleep(250);

                intake.setMode(IntakeMode.STOP);
                intake.update();

                arm.stack();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(27, 45, Math.PI))
                        .addMarker(() -> {
                            claw.intake();

                            arm.intake();

                            grabber.setMode(IntakeMode.PREMATCH);
                            grabber.update();

                            intake.setMode(IntakeMode.PULL);
                            intake.update();

                            return Unit.INSTANCE;
                        })
                        .back(19)
                        .build()
                );

                lopMode.sleep(200);

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 39, Math.PI))
                        .splineTo(new Pose2d(-39, 41, Math.PI))
                        .splineTo(new Pose2d(-55, 19, 1.5*Math.PI))
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(46, 40, Math.PI))
                        .addMarker(new Vector2d(7, 35), () -> {
                            claw.grab();
                            lopMode.sleep(150);
                            intake.setMode(IntakeMode.STOP);
                            intake.update();
                            arm.stack();
                            return Unit.INSTANCE;
                        })
                        .build()
                );

                claw.intake();
                lopMode.sleep(150);
                arm.intake();
                lopMode.sleep(250);
                intake.setMode(IntakeMode.PULL);
                intake.update();claw.intake();
                arm.intake();
                intake.setMode(IntakeMode.PULL);
                intake.update();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-23, 13, 1.5*Math.PI))
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(48, 40, Math.PI))
                        .addMarker(new Vector2d(0, 32), () -> {
                            claw.grab();
                            lopMode.sleep(150);
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
                        .forward(40)
                        .build()
                );
            }
        }

        public void blueQuarryPath3(){
            while(lopMode.opModeIsActive()) {
                arm.setMode(MotorMode.AUTO);
                arm.setPower(0);
                intake.setMaxPower(1.0);
                claw.setMode(ClawMode.AUTO);
                lift.setMode(MotorMode.AUTO);
                drive.setPoseEstimate(new Pose2d(-13, 62.5, 1.5 * Math.PI));

                claw.intake();
                intake.setMode(IntakeMode.OUT);
                intake.update();
                lift.downPosition();

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .splineTo(new Pose2d(-41, 20, 1.5*Math.PI))
                                .build()
                );

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse().splineTo(new Pose2d(0, 39, Math.PI))
                                .splineTo(new Pose2d(52, 22, 0.5*Math.PI))
                                .addMarker(new Vector2d(49, 24), () -> {
                                    grabber.setMode(IntakeMode.OUT);
                                    grabber.update();
                                    claw.grab();
                                    return Unit.INSTANCE;
                                })
                                .lineTo(new Vector2d(52, 17))
                                .build()
                );

                lopMode.sleep(250);

                intake.setMode(IntakeMode.STOP);
                intake.update();

                arm.stack();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(27, 45, Math.PI))
                        .addMarker(() -> {
                            claw.intake();

                            arm.intake();

                            grabber.setMode(IntakeMode.PREMATCH);
                            grabber.update();

                            intake.setMode(IntakeMode.PULL);
                            intake.update();

                            return Unit.INSTANCE;
                        })
                        .back(19)
                        .build()
                );

                lopMode.sleep(200);

//        drive.followTrajectorySync(drive.trajectoryBuilder()
//                .splineTo(new Pose2d(0, -39, Math.PI))
//                .splineTo(new Pose2d(-66, -19, 0.8*Math.PI))
//                .build()
//        );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 39, Math.PI))
                        .splineTo(new Pose2d(-38, 39, Math.PI))
                        .strafeLeft(35)
                        .forward(20)
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .back(12)
                        .strafeRight(45)
                        .reverse().splineTo(new Pose2d(49, 45, Math.PI))
                        .addMarker(new Vector2d(7, 35), () -> {
                            claw.grab();
                            lopMode.sleep(150);
                            intake.setMode(IntakeMode.STOP);
                            intake.update();
                            arm.stack();
                            return Unit.INSTANCE;
                        })
                        .build()
                );

                claw.intake();
                lopMode.sleep(150);
                arm.intake();
                lopMode.sleep(250);
                intake.setMode(IntakeMode.PULL);
                intake.update();claw.intake();
                arm.intake();
                intake.setMode(IntakeMode.PULL);
                intake.update();

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-31, 13, 1.5*Math.PI))
                        .build()
                );

                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .reverse().splineTo(new Pose2d(48, 40, Math.PI))
                        .addMarker(new Vector2d(0, 32), () -> {
                            claw.grab();
                            lopMode.sleep(150);
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
                        .forward(40)
                        .build()
                );
            }
        }
    }
