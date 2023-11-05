package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ThreeRectanglesVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

// Uses custom vision processor  for single camera / 3 rectangles
@Autonomous(name = "Concept: Single Camera", group = "Concept")
public class OneCamera3RectCustomOpmode extends OpMode {
    private ThreeRectanglesVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    @Override
    public void init() {
        visionProcessor = new ThreeRectanglesVisionProcessor(); visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }
    @Override
    public void init_loop() { }
    @Override
    public void start() {
        visionPortal.stopStreaming(); }
    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getSelection()); }
}