package org.firstinspires.ftc.teamcode.PIDControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

@TeleOp(name="LiftPID")
@Config
    public class LiftPIDTuner extends OpMode {

        public static int travel = 90;

        public static double maxPower = 1;

        public static double kp = 9;
        public static double ki = 0.0;
        public static double kd = 0.0;

        public DcMotorEx lift     = null;
        public TouchSensor down = null;
        public TouchSensor up = null;

        public PIDController pid = new PIDController(kp, ki, kd);

        //The gear ratio between the motor and the gear powering the lead screw
        public static double gearRatio = 2;

        //The increase in lead screw position per rotation of the screw
        public double lead = 8;

        //How far the cascading slides can fully extend
        public double fullLiftExtension = 37.25;

        //How much distance between lowest point on the lead screw and the highest point
        public double fullScrewDistance = 9.25;

        //Motor encoder ticks per inch of vertical distance for the slides
        public static double ticksPerInch = 1;

        public double screwMovementPerLiftInch = fullLiftExtension/fullScrewDistance;

        public double screwTicksPerInch = 1;

        public static int start = 0;

        double startMotorPos;

        private int maxPosition = 1000000000;

        private boolean recentlyChanged = false;

        public int loopCount = 0;


        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            lift = (DcMotorEx)hardwareMap.get(DcMotor.class, "lift");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            down = hardwareMap.get(TouchSensor.class, "down");
            up = hardwareMap.get(TouchSensor.class, "up");

            mmToInch(lead);

            screwTicksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/lead;

            ticksPerInch = 240;

            //ticksPerInch = (lift.getMotorType().getTicksPerRev()*gearRatio)/360;

            startMotorPos = lift.getCurrentPosition();
        }

        @Override
        public void loop() {
            pid.setkP(kp);
            pid.setkI(ki);
            pid.setkD(kd);

            lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp, 0, 0, 0));

            telemetry.addData("Distance to travel: ", travel * ticksPerInch);
            telemetry.addData("Ticks Per INCH", ticksPerInch);
            telemetry.addData("Encoder Value: ", lift.getCurrentPosition());
            telemetry.update();

            if (start == 1){
                setPosition(travel);
            }
        }

        public void setPosition(double distance){

            int newAngle = (int)(distance*ticksPerInch);

            if(newAngle >= maxPosition){
                newAngle = maxPosition;
            }

            lift.setTargetPosition(newAngle);

            if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION ){
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

            telemetry.addData("Arm Power", lift.getPower());
            telemetry.update();
            loopCount += 1;
        }

        public void mmToInch(double x){
            x = x/25.4;
        }

    }

