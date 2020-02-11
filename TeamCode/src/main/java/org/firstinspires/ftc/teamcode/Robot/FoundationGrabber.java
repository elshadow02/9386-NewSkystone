package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class FoundationGrabber {
    private Servo foundL = null;
    private Servo foundR = null;
    private OpMode opMode = null;

    private double maxPower = 1;

    private IntakeMode mode = IntakeMode.STOP;

    private Gamepad gamepad = null;

    public FoundationGrabber(HardwareMap hwMap, OpMode opmode){
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

    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, -1, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void update(){

        switch(mode){
            case PREMATCH:
                foundL.setPosition(0.0);
                foundR.setPosition(1.0);
                break;
            case TELEOP:
                if(gamepad.a){
                    foundL.setPosition(1.0);
                    foundR.setPosition(0.0);
                }
                if(gamepad.x){
                    foundL.setPosition(0.0);
                    foundR.setPosition(1.0);
                }
                break;
            case OUT:
                foundL.setPosition(1.0);
                foundR.setPosition(0.0);
                break;
            case STOP:
                //Do nothing
                break;
        }
    }

}