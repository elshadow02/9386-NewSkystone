package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

import static org.firstinspires.ftc.teamcode.RoboMath.MMToInchKt.mmToInch;

public class Lift {
    private DcMotor lift = null;
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
    public double ticksPerInch = 1;

    public double screwMovementPerLiftInch = fullScrewDistance/fullLiftExtension;

    public double screwTicksPerInch = 1;

    public Lift(HardwareMap hwMap, Gamepad pad){
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        down = hwMap.get(TouchSensor.class, "down");
        up = hwMap.get(TouchSensor.class, "up");

        mmToInch(lead);

        screwTicksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/lead;

        ticksPerInch = screwTicksPerInch * screwMovementPerLiftInch;

        gamepad = pad;
    }

    public void setMode(MotorMode mode){
        this.mode = mode;
    }

    public MotorMode getMode(){
        return mode;
    }

    private void setPower(double v){
        lift.setPower(v * maxPower);

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

    private void downPosition(){
        setDistance(0);
    }

    public void nextPosition(){
        setDistance(225);
    }

    public void setDistance(double distance){
        distance *= ticksPerInch;

        double output;

        double error;


        error = distance - lift.getCurrentPosition();

        //output = error * kp;
        output = pid.calculate(error);

        if(!down.isPressed() && !up.isPressed()) {
            lift.setPower(output);
        }
        else if(down.isPressed() && output < 0){
            lift.setPower(0);
        }
        else if(up.isPressed() && output > 0){
            lift.setPower(0);
        }
        else{
            lift.setPower(0);
        }
    }

    public void update(){
        if(gamepad.right_bumper && liftPosition != 8){
            liftPosition += 1;
        }

        if(gamepad.left_bumper && liftPosition != 1){
            liftPosition -= 1;
        }

        if(gamepad.dpad_up){
            setMode(MotorMode.RUN_TO_POSITION);
        }

        if(gamepad.dpad_down){
            liftPosition = 1;
            setMode(MotorMode.RUN_TO_POSITION);
        }

        switch(mode){
            case RUN_TO_POSITION:
                if(liftPosition == 0){
                    downPosition();
                }
                if(liftPosition == 0){
                    nextPosition();
                }
            case CONTROLLED:
                if(down.isPressed() && -gamepad.left_stick_y < 0){
                    lift.setPower(0);
                }
                else if(up.isPressed() && -gamepad.left_stick_y > 0){
                    lift.setPower(0);
                }
                else{
                    lift.setPower(-gamepad.left_stick_y);
                }
            case STOP:
                setPower(0);
        }
    }

}
