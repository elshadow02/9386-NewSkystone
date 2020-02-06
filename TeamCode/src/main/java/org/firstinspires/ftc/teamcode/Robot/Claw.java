package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw = null;
    private ClawMode mode;
    private Gamepad pad = null;

    public Claw(OpMode opmode){
        claw = opmode.hardwareMap.get(Servo.class, "claw");
        pad = opmode.gamepad2;
    }

    public void setMode(ClawMode newMode){
        this.mode = newMode;
    }

    public ClawMode getMode(){
        return this.mode;
    }

    public void grab(){
        claw.setPosition(0.04);
    }

    public void intake(){
        claw.setPosition(0.5);
    }

    public void update(){
        if(getMode() != ClawMode.AUTO) {
            if (pad.x) {
                mode = ClawMode.GRAB;
            }
            if (pad.b) {
                mode = ClawMode.INTAKE;
            }
        }

        switch(mode){
            case GRAB:
                claw.setPosition(0.04);
            case INTAKE:
                claw.setPosition(0.5);
            default:
                //Do nothing
        }
    }
}
