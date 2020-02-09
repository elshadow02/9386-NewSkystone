package org.firstinspires.ftc.teamcode.PIDControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/*
 * Created by Ethan L. 1-28-2020
 *
 * OpMode for tuning the PID for an arm that uses a single motor. Utilizes the Dashboard.
 *
 * Start the program in the Dashboard and edit the variables. Once you are ready to begin, change
 * the start variable to equal 1 and graph the current position against the target (angle). Adjust
 * the PID variables until the current position reaches the target without oscillation.
 */

@TeleOp(name="ArmPID")
@Config
    public class ArmPIDTuner extends OpMode {

        public static int travel = 90;

        public static double maxPower = 1;

        public static double kp = 6;
        public static double ki = 0.0;
        public static double kd = 0.0;

        public DcMotorEx arm      = null;
        public TouchSensor down = null;
        public TouchSensor up = null;

        public PIDController pid = new PIDController(kp, ki, kd);

        //If you are using a gear system between the arm motor and the output shaft,
        //Put the overall gear ratio here.
        public static double gearRatio = 2;

        //Motor encoder ticks per angle of rotation
        public double ticksPerAngle = 1;

        public static int start = 0;

        double startMotorPos;

        public int maxPosition = 1000000;

        public int loopCount = 0;

        public PIDFController pidf = new PIDFController(new PIDCoefficients(kp, ki, kd));

        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ticksPerAngle = (arm.getMotorType().getTicksPerRev()*gearRatio)/360;

            startMotorPos = arm.getCurrentPosition();

            down = hardwareMap.get(TouchSensor.class, "down");
            up = hardwareMap.get(TouchSensor.class, "up");
        }

        @Override
        public void loop() {
            arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp, ki, kd, 0));

            //pidf = new PIDFController(new PIDCoefficients(kp, ki, kd))

            telemetry.addData("Distance to travel: ", travel);
            telemetry.addData("Encoder Value: ", arm.getCurrentPosition());
            telemetry.addData("Error: ", arm.getTargetPosition() - arm.getCurrentPosition());
            telemetry.addData("The Target: ", arm.getTargetPosition());
            telemetry.addData("Arm PID: ", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.update();

            if (start == 1){
                setPosition(travel);
            }
        }

        public void setPosition(double angle){

            int newAngle = (int)(angle*ticksPerAngle);

            if(newAngle >= maxPosition){
                newAngle = maxPosition;
            }

            arm.setTargetPosition(newAngle);

            if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(down.isPressed()){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else if(up.isPressed()){
                maxPosition = arm.getCurrentPosition();
                if(arm.getTargetPosition() < arm.getCurrentPosition()){
                    arm.setPower(0.2);
                }
                else{
                    arm.setPower(0);
                }
            }
            else if ((arm.getTargetPosition() - arm.getCurrentPosition()) < 325){
                arm.setPower(0.2);
            }
            else {
                arm.setPower(maxPower);
            }

            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Arm Velocity", arm.getVelocity());
            telemetry.update();

//            angle *= ticksPerAngle;
//
//            double output;
//
//            double error;
//
//
//            error = angle - (arm.getCurrentPosition() - startMotorPos);
//
//            //output = error * kp;
//            output = pid.calculate(error);
//
//            arm.setPower(output);

//            telemetry.addData("Encoder target: ", angle);
//            telemetry.addData("Error: ", pidf.getLastError());
//            telemetry.addData("kp: ", kp);
//            telemetry.addData("ki: ", ki);
//            telemetry.addData("kd: ", kd);
//            telemetry.addData("Target: ", pidf.getTargetPosition());
//            telemetry.update();
            loopCount += 1;
        }

    }

