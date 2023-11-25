package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.DrawRectangleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

// Uses custom vision processor  for 2 camera camera / 1 rectangle each
@Disabled
@TeleOp(name = "Concept: Double Cameras", group = "Concept")
public class TwoCameraCustomProcessorOpMode extends LinearOpMode {
    private DrawRectangleProcessor visionProcessor;
    private VisionPortal visionPortal, vp;
    private WebcamName webcam1, webcam2;

    private boolean oldDpadLeft;
    private boolean oldDpadRight;


    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize vision
        initDoubleVision();

        if (visionPortal == null) {
            telemetry.addData("vision portal"," not initialized properly");
        }
        //check Camera streams
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryCameraSwitching();


                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad2.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad2.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                doCameraSwitching();

                // Share the CPU.
                sleep(20);
            } //end while
        } //  if
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()


    private void telemetryCameraSwitching() {

        if (visionPortal.getActiveCamera().equals(webcam1)) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addData("Detection for Webcam 1: ", visionProcessor.getSelection());
            telemetry.addData("Press DPAD RIGHT", "to switch to Webcam 2");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addData("Detection for Webcam 2: ", visionProcessor.getSelection());
            telemetry.addData("Press DPAD LEFT", "to switch to Webcam 1");
        }

    }   // end method telemetryCameraSwitching()

    private void doCameraSwitching() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // If the left dpad is pressed, use Webcam 1.
            // If the right dpad is pressed, use Webcam 2.
            boolean newDpadLeft = gamepad2.dpad_left;
            boolean newDpadRight = gamepad2.dpad_right;
            if (newDpadLeft && !oldDpadLeft) {
                visionPortal.setActiveCamera(webcam1);
            } else if (newDpadRight && !oldDpadRight) {
                visionPortal.setActiveCamera(webcam2);
            }
            oldDpadLeft = newDpadLeft;
            oldDpadRight = newDpadRight;
        }

    }   // end method doCameraSwitching()

    /**
     * Initialize both webcams.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        //  Configuration
        // -----------------------------------------------------------------------------------------
        visionProcessor = new DrawRectangleProcessor();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(visionProcessor)
                .build();
        // Set the webcam 1 as active camera first
        //visionPortal.setActiveCamera(webcam1);
    }   // end initDoubleVision()
}