package org.firstinspires.ftc.teamcode.PIDControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.RoboMath.MMToInchKt.mmToInch;

/*
 * Created by Ethan L. 1-28-2020
 *
 * OpMode for tuning the PID for the Skystone lift that uses a single motor attached to a lead screw
 * that powers a cascading lift. Utilizes the Dashboard.
 *
 * Start the program in the Dashboard and edit the variables. Once you are ready to begin, change
 * the start variable to equal 1 and graph the current position against the target (distance).
 * Adjust the PID variables until the current position reaches the target without oscillation.
 */

@TeleOp(name="PIDTuner")
@Config
    public class LiftPIDTuner extends OpMode {

        public static int travel = 90;

        public static double maxPower = 1;

        public static double kp = 1;
        public static double ki = 0.0;
        public static double kd = 0.0;

        public DcMotor lift     = null;
        public TouchSensor down = null;
        public TouchSensor up = null;

        public PIDController pid = new PIDController(kp, ki, kd);

        //The gear ratio between the motor and the gear powering the lead screw
        public static double gearRatio = 1;

        //The increase in lead screw position per rotation of the screw
        public double lead = 8;

        //How far the cascading slides can fully extend
        public double fullLiftExtension = 72;

        //How much distance between lowest point on the lead screw and the highest point
        public double fullScrewDistance = 14;

        //Motor encoder ticks per inch of vertical distance for the slides
        public double ticksPerInch = 1;

        public double screwMovementPerLiftInch = fullScrewDistance/fullLiftExtension;

        public double screwTicksPerInch = 1;

        public static int start = 0;

        double startMotorPos;

        public int loopCount = 0;


        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            down = hardwareMap.get(TouchSensor.class, "down");
            up = hardwareMap.get(TouchSensor.class, "up");

            mmToInch(lead);

            screwTicksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/lead;

            ticksPerInch = screwTicksPerInch * screwMovementPerLiftInch;

            ticksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/360;

            startMotorPos = lift.getCurrentPosition();
        }

        @Override
        public void loop() {
            pid.setkP(kp);
            pid.setkI(ki);
            pid.setkD(kd);

            telemetry.addData("Distance to travel: ", travel);
            telemetry.addData("Encoder Value: ", lift.getCurrentPosition());
            telemetry.update();

            if (start == 1){
                setPosition(travel);
            }
        }

        public void setPosition(double distance){
            distance *= ticksPerInch;

            double output;

            double error;


            error = distance - (lift.getCurrentPosition() - startMotorPos);

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
                telemetry.addLine("Argh! SOmethign wrng");
                telemetry.update();
            }

            telemetry.addData("Encoder target: ", distance);
            telemetry.addData("Error: ", error);
            telemetry.addData("Output: ", output);
            telemetry.addData("kp: ", kp);
            telemetry.addData("ki: ", ki);
            telemetry.addData("kd: ", kd);
            telemetry.addData("loop count: ", loopCount);
            telemetry.update();
            loopCount += 1;
        }

    }

