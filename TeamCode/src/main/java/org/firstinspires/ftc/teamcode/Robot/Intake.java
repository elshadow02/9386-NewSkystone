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
    private OpMode opMode = null;
    private double intakeLeftStartPos = 0;

    private PIDController pid = new PIDController(0, 0, 0);

    private static double gearRatio = 1;

    //Motor encoder ticks per angle of rotation
    private double ticksPerAngle = 1;

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

        ticksPerAngle = (intakeLeft.getMotorType().getTicksPerRev()*gearRatio)/360;

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
                setPower(0);
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
            case OUT:

            case STOP:
                setPower(0);
        }
    }

}