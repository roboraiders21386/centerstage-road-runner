package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.VisionOpenCVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="Concept: CustomOpenCVAuto ", group = "Concept")
public class CustomOpenCVAuto extends LinearOpMode {
    OpenCvCamera camera;
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        //ColorBlobDetector detector = new ColorBlobDetector(telemetry);
        VisionOpenCVPipeline detector = new VisionOpenCVPipeline(telemetry);

        //TEst for RED or BLUE
        detector.setAlliancePipe("BLUE");
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                //camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);  // this was used by ColorBlobAuto
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                while (!detector.hasProcessedFrame) sleep(50);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            //telemetry.clearAll();
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Park Position Identified by Camera: ", detector.whichSide);
            telemetry.update();
        }

        /*
        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }*/

        camera.stopStreaming();
    }
}