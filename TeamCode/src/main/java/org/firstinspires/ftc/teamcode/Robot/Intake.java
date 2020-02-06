package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

public class Intake {
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;
    private Servo intakeLeftServo = null;
    private Servo intakeRightServo = null;
    private Servo foundL = null;
    private Servo foundR = null;
    private OpMode opMode = null;

    private double maxPower = 1;

    private IntakeMode mode = IntakeMode.STOP;

    private Gamepad gamepad = null;

    public Intake(HardwareMap hwMap, OpMode opmode){
        intakeLeft = hwMap.get(DcMotor.class, "iL");
        intakeRight = hwMap.get(DcMotor.class, "iR");
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeftServo = hwMap.get(Servo.class, "iLS");
        intakeRightServo = hwMap.get(Servo.class, "iRS");
        foundL = hwMap.get(Servo.class, "foundL");
        foundR = hwMap.get(Servo.class, "foundR");

        this.opMode = opmode;

        this.gamepad = opmode.gamepad1;
    }

    public void setMode(IntakeMode mode){
        this.mode = mode;
    }

    public IntakeMode getMode(){
        return mode;
    }

    private void setPower(double v){
        intakeLeft.setPower(v * maxPower);
        intakeRight.setPower(v * maxPower);
    }

    private void setPosition(){
        intakeLeftServo.setPosition(0);
        intakeRightServo.setPosition(0);
    }

    public double getPower(){
        return intakeLeft.getPower();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, -1, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void update(){

        switch(mode){
            case PREMATCH:
                intakeLeftServo.setPosition(0.3);
                intakeRightServo.setPosition(1.0);
                setPower(0);
                break;
            case TELEOP:
                if (gamepad.right_trigger > 0.1){
                    setPower(gamepad.right_trigger);
                }
                else if (gamepad.left_trigger > 0.1){
                    setPower(-gamepad.left_trigger);
                }
                else{
                    setPower(0);
                }

                if(gamepad.dpad_right){
                    intakeLeftServo.setPosition(0.3);
                    intakeRightServo.setPosition(1.0);
                }
                if(gamepad.dpad_left){
                    intakeLeftServo.setPosition(0.3);
                    intakeRightServo.setPosition(1.0);
                }
                break;
            case OUT:
                intakeLeftServo.setPosition(0.3);
                intakeRightServo.setPosition(1.0);
                setPower(1.0);
                break;
            case STOP:
                setPower(0);
                break;
        }
    }

}