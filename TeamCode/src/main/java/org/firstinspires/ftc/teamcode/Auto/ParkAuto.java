package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MintyPancho;
import org.firstinspires.ftc.teamcode.Robot.RobotMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.teamcode.VuforiaTesting.VuforiaImpPlus;

@Autonomous(name="RedQuarryAuto", group ="Concept")
public class ParkAuto extends LinearOpMode {

    //EEHardware bot = new EEHardware();

    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor wheel4;

    public static double DRIVE_kP = 0.001;
    public static double DRIVE_kI = 0.0000007;
    public static double DRIVE_kD = 1;

    public static double TURN_kP = 0.05;  // increase this number to increase responsiveness. decrease this number to decrease oscillation
    public static double TURN_kI = 0.01;  // increase this number to decrease steady state error (controller stops despite error not equalling 0)
    public static double TURN_kD = 0.025;

    double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    double TICKS_PER_ROTATION = 537.6;
    double TICKS_PER_INCH      = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    private ElapsedTime runtime = new ElapsedTime();

    public int SkystonePos = 1; //Position of Skystone: 1 = Left; 2 = middle; 3 = right.

    @Override
    public void runOpMode() throws InterruptedException {
        MintyPancho robot = new MintyPancho(this);

        robot.setMode(RobotMode.STOP);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.addData("robot mode", robot.getMode());
            telemetry.addData("lift position", robot.liftPos());
            telemetry.addData("robot mode 2", robot.getMode());
            telemetry.addData("Width", cols);
            telemetry.addData("Width", cols);
            telemetry.addData("Width", cols);



            System.out.println("Camera color: " + valLeft+"   "+valMid+"   "+valRight);

            telemetry.update();
            sleep(100);
        }

        robot.setMode(RobotMode.PARK);
        robot.update();
    }
}