package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

public class Arm {
    private DcMotor arm = null;
    private OpMode opMode = null;
    private double armStartPos = 0;

    private PIDController pid = new PIDController(0, 0, 0);

    private static double gearRatio = 1;

    //Motor encoder ticks per angle of rotation
    private double ticksPerAngle = 1;

    private double maxPower = 1;

    private Modes.MotorMode mode = Modes.MotorMode.STOP;

    private Gamepad gamepad = null;

    private boolean armIntakePosition = true;

    public Arm(HardwareMap hwMap, Gamepad pad){
        arm = hwMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerAngle = (arm.getMotorType().getTicksPerRev()*gearRatio)/360;

        gamepad = pad;
    }

    public void setMode(Modes.MotorMode mode){
        this.mode = mode;
    }

    public Modes.MotorMode getMode(){
        return mode;
    }

    private void setPower(double v){
        arm.setPower(v * maxPower);

    }

    public double getPower(){
        return arm.getPower();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, -1, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    private void intake(){
        setAngle(0);
    }

    public void stack(){
        setAngle(225);
    }

    public void setAngle(double angle){
        angle *= ticksPerAngle;

        double output;

        double error;


        error = angle - arm.getCurrentPosition();

        //output = error * kp;
        output = pid.calculate(error);

        arm.setPower(output * maxPower);
    }

    public void update(){
        if(gamepad.dpad_right){
            setMode(Modes.MotorMode.RUN_TO_POSITION);
            armIntakePosition = true;
        }

        if(gamepad.dpad_left){
            setMode(Modes.MotorMode.RUN_TO_POSITION);
            armIntakePosition = false;
        }

        switch(mode){
            case RUN_TO_POSITION:
                if(armIntakePosition == true){
                    intake();
                }
                if(armIntakePosition == false){
                    stack();
                }
            case CONTROLLED:
                setPower(-gamepad.right_stick_y);
            case STOP:
                setPower(0);
        }
    }

}
