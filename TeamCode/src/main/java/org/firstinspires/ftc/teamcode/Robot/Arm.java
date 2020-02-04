package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

public class Arm {
    private DcMotorEx arm = null;
    private OpMode opMode = null;
    private double armStartPos = 0;

    private static double gearRatio = 2;

    //Motor encoder ticks per angle of rotation
    private double ticksPerAngle = 1;

    private double maxPower = 0.5;

    private MotorMode mode = MotorMode.STOP;

    private Gamepad gamepad = null;

    private boolean armIntakePosition = true;

    public Arm(HardwareMap hwMap, OpMode opmode){
        arm = (DcMotorEx)hwMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerAngle = (arm.getMotorType().getTicksPerRev()*gearRatio)/360;

        this.opMode = opmode;

        this.gamepad = opmode.gamepad2;

        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(6, 0, 0, 0));
    }

    public void setMode(MotorMode mode){
        this.mode = mode;
    }

    public MotorMode getMode(){
        return mode;
    }

    public void setPower(double v){
        arm.setPower(v);

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

    public void intake(){
        setAngle(0);
    }

    public void stack(){
        setAngle(180);
    }

    public void setAngle(double angle){
        if(getMode() == MotorMode.RUN_TO_POSITION || getMode() == MotorMode.AUTO) {
            int newAngle = (int) (angle * ticksPerAngle);

            arm.setTargetPosition(newAngle);

            if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if ((arm.getTargetPosition() - arm.getCurrentPosition()) < 325) {
                arm.setPower(0.2);
            } else {
                arm.setPower(maxPower);
            }
        }
    }

    public void update(){
        if(gamepad.dpad_right){
            setMode(MotorMode.RUN_TO_POSITION);
            armIntakePosition = true;
        }

        if(gamepad.dpad_left){
            setMode(MotorMode.RUN_TO_POSITION);
            armIntakePosition = false;
        }

        if(Math.abs(gamepad.right_stick_y) > 0.1 && mode != MotorMode.CONTROLLED){
            mode = MotorMode.CONTROLLED;
        }

        switch(mode){
            case AUTO:

            case RUN_TO_POSITION:
                if(armIntakePosition == true){
                    intake();
                }
                if(armIntakePosition == false){
                    stack();
                }
                break;
            case CONTROLLED:
                if (arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                setPower(-gamepad.right_stick_y * maxPower);
                break;
            case STOP:
                if (arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                setPower(0);
                break;
        }
    }

}
