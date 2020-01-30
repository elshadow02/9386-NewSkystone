package org.firstinspires.ftc.teamcode.Robot;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class DriveTrain extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;
    private DriveMode driveMode = DriveMode.STOP;
    //private Gamepad gamepad = null;
    private OpMode opMode = null;
    private double forward, strafe, rotate;
    private double frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
    private double speedAngle, joystickAngle, angleChange;
    private double maxPower = 1;

    public DriveTrain(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "fL");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "bL");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "bR");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "fR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public DriveTrain(HardwareMap hardwareMap, OpMode opmode) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "fL");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "bL");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "bR");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "fR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        this.opMode = opmode;
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public DriveMode getDriveMode(){
        return this.driveMode;
    }

    public void setMaxPower(double pow){
        this.maxPower = pow;
    }

    public double getMaxPower(){
        return this.maxPower;
    }

    public void stateUpdate(){
        switch(driveMode){
            case MECANUM:
                rotate = opMode.gamepad1.right_stick_x;
                forward = -opMode.gamepad1.left_stick_y;
                strafe = opMode.gamepad1.left_stick_x;

                frontLeftSpeed = forward + rotate + strafe;
                frontRightSpeed = forward - rotate - strafe;
                backLeftSpeed = forward + rotate - strafe;
                backRightSpeed = forward - rotate + strafe;

                double max = Math.abs(frontLeftSpeed);
                if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
                if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
                if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);

                if (max > 1) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                setMotorPowers(frontLeftSpeed, backLeftSpeed, backRightSpeed, frontRightSpeed);
            case TANK:
                setMotorPowers(-opMode.gamepad1.left_stick_y, -opMode.gamepad1.left_stick_y, -opMode.gamepad1.right_stick_y, -opMode.gamepad1.right_stick_y);
            case COOL_MECANUM:
                joystickAngle = Math.atan2(-opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x);

                //Defines angleChange variable as the x-value received from the imu.
                angleChange = imu.getAngularOrientation().firstAngle;
                //Assigns speedAngle as the difference between joystickAngle and angleChange.
                speedAngle = joystickAngle - angleChange;

                //Gets the rotational value for our drive.
                rotate = opMode.gamepad1.right_stick_x;

                double dampner;

                if (Math.abs(opMode.gamepad1.left_stick_x) > Math.abs(opMode.gamepad1.left_stick_y)) {
                    dampner = Math.abs(opMode.gamepad1.left_stick_x);
                } else if (Math.abs(opMode.gamepad1.left_stick_y) > Math.abs(opMode.gamepad1.left_stick_x)) {
                    dampner = Math.abs(opMode.gamepad1.left_stick_y);
                } else {
                    dampner = Math.abs(opMode.gamepad1.left_stick_y);
                }

                //Make sure that the absolute value of either the y- or x-value of the joystick is greater than 0.1.
                //Because speedAngle will never produce an angle where the sine and cosine of that angle equals 0,
                //we need to make sure that the robot does not move unless the joystick is pressed.
                if (Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.left_stick_x) > 0.1) {

                    forward = Math.sin(speedAngle);
                    strafe = Math.cos(speedAngle);

                    frontRightSpeed = forward - rotate - strafe;
                    frontLeftSpeed = forward + rotate + strafe;
                    backLeftSpeed = forward + rotate - strafe;
                    backRightSpeed = forward - rotate + strafe;

                    max = Math.abs(frontLeftSpeed);
                    if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
                    if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
                    if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);

                    if (max > 1) {
                        frontLeftSpeed /= max;
                        frontRightSpeed /= max;
                        backLeftSpeed /= max;
                        backRightSpeed /= max;
                    }

                    setMotorPowers(frontLeftSpeed, backLeftSpeed, backRightSpeed, frontRightSpeed);
                } else {
                    setMotorPowers(rotate, rotate, -rotate, -rotate);
                }

            case TURTLE_SPEED:
                rotate = opMode.gamepad1.right_stick_x;
                forward = -opMode.gamepad1.left_stick_y;
                strafe = opMode.gamepad1.left_stick_x;

                frontLeftSpeed = forward + rotate + strafe;
                frontRightSpeed = forward - rotate - strafe;
                backLeftSpeed = forward + rotate - strafe;
                backRightSpeed = forward - rotate + strafe;

                max = Math.abs(frontLeftSpeed);
                if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
                if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
                if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);

                if (max > 1) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                setMotorPowers(frontLeftSpeed * 0.2, backLeftSpeed * 0.2, backRightSpeed * 0.2, frontRightSpeed * 0.2);
            case STOP:
                setMotorPowers(0, 0, 0, 0);
        }
    }
}
