package org.firstinspires.ftc.teamcode.ComputerVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import android.util.Log;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.Init3BlockDetection;
import org.firstinspires.ftc.teamcode.ComputerVision.VisionPipelineFrom16626;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Vision16626Test", group="vision")
public class Vision16626Test extends LinearOpMode {
    public Init3BlockDetection pipeline;

    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("pos", 0);
        telemetry.update();
        pipeline = new VisionPipelineFrom16626();
        telemetry.addData("pos", 1);
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        telemetry.addData("pos", 2);
        telemetry.update();

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry.addData("pos", 3);
        telemetry.update();
        camera.openCameraDevice();
        telemetry.addData("pos", 4);
        telemetry.update();
        camera.setPipeline(pipeline);
        telemetry.addData("pos", 5);
        telemetry.update();
        camera.startStreaming(pipeline.getWidth(), pipeline.getHeight(), OpenCvCameraRotation.UPRIGHT);

        while (!opModeIsActive() && !isStopRequested()){
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

        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
