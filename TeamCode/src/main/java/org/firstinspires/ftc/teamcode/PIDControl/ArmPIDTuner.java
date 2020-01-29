package org.firstinspires.ftc.teamcode.PIDControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        public static double kp = 1;
        public static double ki = 0.0;
        public static double kd = 0.0;

        public DcMotor arm      = null;

        public PIDController pid = new PIDController(kp, ki, kd);

        //If you are using a gear system between the arm motor and the output shaft,
        //Put the overall gear ratio here.
        public static double gearRatio = 1;

        //Motor encoder ticks per angle of rotation
        public double ticksPerAngle = 1;

        public static int start = 0;

        double startMotorPos;

        public int loopCount = 0;


        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ticksPerAngle = (arm.getMotorType().getTicksPerRev()*gearRatio)/360;

            startMotorPos = arm.getCurrentPosition();
        }

        @Override
        public void loop() {
            pid.setkP(kp);
            pid.setkI(ki);
            pid.setkD(kd);

            telemetry.addData("Distance to travel: ", travel);
            telemetry.addData("Encoder Value: ", arm.getCurrentPosition());
            telemetry.update();

            if (start == 1){
                setPosition(travel);
            }
        }

        public void setPosition(double angle){
            angle *= ticksPerAngle;

            double output;

            double error;


            error = angle - (arm.getCurrentPosition() - startMotorPos);

            //output = error * kp;
            output = pid.calculate(error);

            arm.setPower(output);

            telemetry.addData("Encoder target: ", angle);
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

