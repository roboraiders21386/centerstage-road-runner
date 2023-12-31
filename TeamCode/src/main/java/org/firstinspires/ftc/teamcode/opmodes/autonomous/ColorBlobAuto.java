package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ColorBlobDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous(name="Concept: Color Blob detector ", group = "Concept")
public class ColorBlobAuto extends LinearOpMode {
    OpenCvCamera camera;
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        ColorBlobDetector detector = new ColorBlobDetector(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();
        /*
        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            //telemetry.clearAll();
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Park Position Identified by Camera: ", detector.getLocation());
            telemetry.update();
        }*/
        telemetry.addData("---------------------------------------","");
        telemetry.addData("Park Position Identified by Camera: ", detector.getLocation());
        telemetry.update();
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