package org.firstinspires.ftc.teamcode.ComputerVision;

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
        pipeline = new VisionPipelineFrom16626();
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
