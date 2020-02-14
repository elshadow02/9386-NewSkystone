package org.firstinspires.ftc.teamcode.ComputerVision;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Roadrunner.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="ConstantCVTestNew", group="vision")
public class NewConstantCVTest extends LinearOpMode {
    public static double DISTANCE = 60;

    public Init3BlockDetection pipeline;

    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        Trajectory traj = drive.trajectoryBuilder()
                .back(DISTANCE)
                .build();

        pipeline = new VisionPipelineFrom16626();
        //pipeline = new VisionPipelineFrom16626();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(pipeline.getWidth(), pipeline.getHeight(), OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Skystone", pipeline.getDetectedSkystonePosition());
            telemetry.addData("Skystone Positions",
                    pipeline.getSkystonePositions(3)[0] + "" + pipeline.getSkystonePositions(3)[1]);
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());

            telemetry.update();
            sleep(10);
        }

        while (opModeIsActive()) {
            drive.followTrajectory(trajectory);
            while (drive.isBusy() && opModeIsActive()) {
                telemetry.addData("Skystone", pipeline.getDetectedSkystonePosition());
                telemetry.addData("Skystone Positions",
                        pipeline.getSkystonePositions(3)[0] + "" + pipeline.getSkystonePositions(3)[1]);
                telemetry.update();

                if (isStopRequested()) return;

                drive.update();
            }

            sleep(1000);

            drive.followTrajectory(traj);
            while (drive.isBusy() && opModeIsActive()) {
                telemetry.addData("Skystone", pipeline.getDetectedSkystonePosition());
                telemetry.addData("Skystone Positions",
                        pipeline.getSkystonePositions(3)[0] + "" + pipeline.getSkystonePositions(3)[1]);
                telemetry.update();

                if (isStopRequested()) return;

                drive.update();
            }

            sleep(1000);

            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
