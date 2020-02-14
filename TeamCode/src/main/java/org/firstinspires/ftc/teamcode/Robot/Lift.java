package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

public class Lift {
    private DcMotorEx lift = null;
    public TouchSensor down = null;
    public TouchSensor up = null;
    private OpMode opMode = null;
    private double liftStartPos = 0;

    private PIDController pid = new PIDController(0, 0, 0);

    private static double gearRatio = 1;

    private double maxPower = 1;

    private MotorMode mode = MotorMode.STOP;

    private Gamepad gamepad = null;

    private int liftPosition = 1;

    public double lead = 8;

    //How far the cascading slides can fully extend
    public double fullLiftExtension = 72;

    //How much distance between lowest point on the lead screw and the highest point
    public double fullScrewDistance = 14;

    //Motor encoder ticks per inch of vertical distance for the slides
    public int ticksPerInch = 240;

    public double screwMovementPerLiftInch = fullScrewDistance/fullLiftExtension;

    public double screwTicksPerInch = 1;

    private int maxPosition = 36*ticksPerInch;

    private boolean recentlyChanged = false;
    private boolean changed = false, changed2 = false;

    public Lift(HardwareMap hwMap, OpMode opmode){
        lift = (DcMotorEx)hwMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        down = hwMap.get(TouchSensor.class, "down");
        up = hwMap.get(TouchSensor.class, "up");

        mmToInch(lead);

        screwTicksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/lead;

        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(9, 0, 0, 0));

        this.opMode = opmode;

        gamepad = opmode.gamepad2;
    }

    public void setMode(MotorMode mode){
        this.mode = mode;
    }

    public MotorMode getMode(){
        return mode;
    }

    public void setPower(double v){
        lift.setPower(v * maxPower);

    }

    public void reset(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getLiftPosition(){
        return liftPosition;
    }

    public double getPower(){
        return lift.getPower();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, -1, 1);
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void downPosition(){
        setDistance(0);
    }

    public boolean isDown(){
        return down.isPressed();
    }

    public void nextPosition(){
        setDistance(liftPosition*4);
    }

    public void initPosition() { setDistance(1);}

    public boolean isBusy(){ return lift.isBusy(); }

    public void setDistance(double distance){
        if(getMode() == MotorMode.RUN_TO_POSITION || getMode() == MotorMode.AUTO) {
            int newAngle = (int) (distance * ticksPerInch);

            if(newAngle >= maxPosition){
                newAngle = maxPosition;
            }

            lift.setTargetPosition(newAngle);

            if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(down.isPressed() && !recentlyChanged){
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                recentlyChanged = true;
            }
            else if(up.isPressed()){
                maxPosition = lift.getCurrentPosition();
                if(lift.getTargetPosition() < lift.getCurrentPosition()){
                    lift.setPower(0.2);
                }
                else{
                    lift.setPower(0);
                }
            }
//            else if ((lift.getTargetPosition() - lift.getCurrentPosition()) < 75){
//                lift.setPower(1.0);
//                recentlyChanged = false;
//            }
            else {
                lift.setPower(maxPower);
                if(!down.isPressed()) {
                    recentlyChanged = false;
                }
            }
        }
    }

    public void update(){
        if (gamepad.right_bumper && liftPosition != 7 && !changed) {
            liftPosition += 1;
            changed = true;
        }
        else if(!gamepad.right_bumper && changed){
            changed = false;
        }

        if (gamepad.left_bumper && liftPosition != 0 && !changed2) {
            liftPosition -= 1;
            changed2 = true;
        }
        else if(!gamepad.left_bumper && changed2){
            changed2 = false;
        }

        if(getMode() != MotorMode.AUTO || getMode() != MotorMode.STOP) {
            if (gamepad.dpad_up) {
                setMode(MotorMode.RUN_TO_POSITION);
            }

            if (gamepad.dpad_down) {
                liftPosition = 0;
                setMode(MotorMode.RUN_TO_POSITION);
            }

            if (Math.abs(gamepad.left_stick_y) > 0.1 && mode != MotorMode.CONTROLLED) {
                mode = MotorMode.CONTROLLED;
            }
        }

        switch(mode) {
            case RUN_TO_POSITION:
                if (liftPosition == 0) {
                    downPosition();
                }
                if (liftPosition > 0) {
                    nextPosition();
                }
                break;
            case CONTROLLED:
                if(lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (down.isPressed() && -gamepad.left_stick_y < 0) {
                    lift.setPower(0);
                } else if (up.isPressed() && -gamepad.left_stick_y > 0) {
                    lift.setPower(0);
                } else {
                    lift.setPower(-gamepad.left_stick_y * maxPower);
                }
                break;
            case PREMATCH:
                initPosition();
                break;
            case STOP:
                setPower(0);
                break;
            case AUTO:
                //do nothing
                break;
        }
    }

    public void mmToInch(double x){
        x = x / 25.4;
    }

}
