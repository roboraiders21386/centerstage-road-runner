/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Autonomous  for only vision detection using OpenCV VisionPortal and park
 */
@Disabled
@Autonomous(name = "RR Auto - Meet 3 No Extras", group = "00-Autonomous", preselectTeleOp = "RR TeleOp - Meet 3")
public class RRAutonmousMeet3NoExtras extends LinearOpMode {

    public static String TEAM_NAME = "RoboRaiders"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 21386; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private VisionOpenCV visionOpenCV;

    public Servo intake;
    public Servo wrist;

    //WRIST parameters
    private double TURN_WRIST = 0.45; //turn it forward
    private double RESET_WRIST = 0.2; //so it doesn't swing 180 back
    private double MOVE_SLIGHTLY = 0.3;

    private double WRIST_SERVO_MAX = 0.8; //1; //0.9;
    private double WRIST_SERVO_MIN = 0.2; //0.5;
    final double CLAW_RELEASE = 1;
    final double CLAW_GRAB = 0;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    ColorSensor colorSensor;    // Hardware Device Object
    LED RLED, LLED;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(Servo.class, "INTAKE");
        wrist = hardwareMap.get(Servo.class, "WRIST");

        //Color Sensor and LED
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        RLED = hardwareMap.get(LED.class, "RLED");
        LLED = hardwareMap.get(LED.class, "LLED");

        //Switch off LEDs
        RLED.enableLight(false); LLED.enableLight(false);


        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses Open CV Vision processor for Team Element detection
        initOpenCV();

        // Wait for the DS start button to be touched.
        telemetry.addLine("Open CV Vision for Red/Blue Team Element Detection");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
                "and along the edge farther from the truss legs. ");
        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
                "We assumed the camera to be in the center of the robot. ");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Open CV Object Detection and keep watching for the Team Element on the spike marks.
            runOpenCVObjectDetection();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d dropYellowPixelPosea = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        Pose2d startWhite = new Pose2d(0,0, 0);
        Pose2d getClose = new Pose2d(0,0, 0);
        Pose2d stackGo = new Pose2d(0,0, 0);
        Pose2d placeWhite = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        switch (startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(24, 14, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(19, 37 , Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(19, 34, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(25, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(30, 37,  Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(30, 34, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(24, -4, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(39, 37, Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(39, 34, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(15, 14, Math.toRadians(-45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d( 8, 33, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(29, 5, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(41, -37, Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(41, -35, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(27, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(32, -35,  Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(32, -33, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27, -12, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(23, -35, Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(23, -33, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, -36, Math.toRadians(90));
                startWhite = new Pose2d(66, -24, Math.toRadians(0));
                getClose = new Pose2d(66,60, Math.toRadians(90));
                stackGo = new Pose2d(64, 91, Math.toRadians(-45));
                placeWhite = new Pose2d(66, -32, Math.toRadians(48));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(23, 6, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(23, 89, Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(23, 87, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(27, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(31, 89, Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(31, 87, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(23, -10, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(35, 89, Math.toRadians(-90));
                        dropYellowPixelPosea = new Pose2d(35, 87, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(20, -12, Math.toRadians(-90));
                intakeStack = new Pose2d(60, -15,Math.toRadians(-90));
                midwayPose2 = new Pose2d(60, 67, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(21, 9, Math.toRadians(10));
                        dropYellowPixelPose = new Pose2d(41, -89, Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(41, -87, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(28, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(35, -90, Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(35, -87, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27, -5, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(25, -89, Math.toRadians(90));
                        dropYellowPixelPosea = new Pose2d(25, -86, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                intakeStack = new Pose2d(64, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(64, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, -84, Math.toRadians(90));
                break;
        }

        //Initialize claw and wrist
        intake.setDirection(Servo.Direction.REVERSE);
        intake.setPosition(CLAW_GRAB); // made it 1 on 1/1/2024
        sleep(300);
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(RESET_WRIST);
        safeWaitSeconds(1);

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        safeWaitSeconds(0.2);
        wrist.setPosition(MOVE_SLIGHTLY);
        safeWaitSeconds(0.2);

        //TODO : Code to drop Purple Pixel on Spike Mark
        //Turn the wrist
        if (startPosition == START_POSITION.BLUE_LEFT ||
                startPosition == START_POSITION.RED_RIGHT) {
            safeWaitSeconds(1);
            wrist.setPosition(TURN_WRIST);
            safeWaitSeconds(1);
        }

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());



        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
                startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());



            //TODO : Code to intake pixel from stack


            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());

            safeWaitSeconds(0.2);
            wrist.setPosition(TURN_WRIST);
            safeWaitSeconds(0.3);
        }

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        //.splineToLinearHeading(dropYellowPixelPosea,0)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        //Claw release
        safeWaitSeconds(2);
        intake.setDirection(Servo.Direction.REVERSE);
        intake.setPosition(CLAW_RELEASE); // made it 1 on 1/1/2024
        safeWaitSeconds(1);


        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO move backwards then lower wrist
                        // TODO after that, strafe left to park
                        .splineToLinearHeading(dropYellowPixelPosea, 0)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(startWhite.position, startWhite.heading)//Math.tan(Math.toRadians(-89)))
                        .strafeToLinearHeading(getClose.position, startWhite.heading)//Math.tan(Math.toRadians(89)))
                        //.turn(90)
                        .build());

        //intake.setPosition(CLAW_RELEASE);
        safeWaitSeconds(0.5);

        //Get stack and go back
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackGo.position, stackGo.heading)//Math.tan(Math.toRadians(89)))
                        .build());

        //TODO open claw here and intake 2 pixels
        wrist.setPosition(RESET_WRIST);
        intake.setPosition(CLAW_GRAB);
        wrist.setPosition(MOVE_SLIGHTLY);
        safeWaitSeconds(0.1);

        //Place the pixels in the park pose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(placeWhite.position, placeWhite.heading  )//Math.tan(Math.toRadians(-89)))
                        //.lineToYConstantHeading(parkPose.position.y)
                        .build());

        //TODO release claw
        intake.setPosition(CLAW_RELEASE);
        safeWaitSeconds(0.5);

        //TODO Repeat this process as much as possible
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing Autonomous adopted for:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the Open CV Object Detection processor.
     */
    public Rect rectLeftOfCameraMid, rectRightOfCameraMid;
    private void initOpenCV() {
        visionOpenCV = new VisionOpenCV(hardwareMap);

        if (startPosition == START_POSITION.RED_LEFT ||
                startPosition == START_POSITION.BLUE_LEFT) {
            rectLeftOfCameraMid = new Rect(10, 240, 150, 240);
            rectRightOfCameraMid = new Rect(160, 240, 470, 160);
        } else { //RED_RIGHT or BLUE_RIGHT
            rectLeftOfCameraMid = new Rect(10, 240, 470, 160);
            rectRightOfCameraMid = new Rect(480, 240, 150, 240);
        }
    }

    /**
     * Add telemetry about Object Detection recognitions.
     */
    private void runOpenCVObjectDetection() {
        visionOpenCV.getSelection();
        telemetry.addData("Identified Parking Location", identifiedSpikeMarkLocation);
        telemetry.addLine("----- Sat values for Team Element -----");
        telemetry.addData("Sat LEFT Of CameraMid: ", visionOpenCV.satRectLeftOfCameraMid);
        telemetry.addData("Sat RIGHT Of CameraMid: ", visionOpenCV.satRectRightOfCameraMid);
        telemetry.addData("SatRectNone Threshold: ", visionOpenCV.satRectNone);
        telemetry.update();
    }

    public class VisionOpenCV implements VisionProcessor {

        CameraSelectedAroundMid selectionAroundMid = CameraSelectedAroundMid.NONE;

        public VisionPortal visionPortal;

        Mat submat = new Mat();
        Mat hsvMat = new Mat();

        public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
        public double satRectNone = 45.0; //Changed from 40 on 1/2/2024 because of camera issue //Tried with 45 - but BLUE_RIGHT is borderline Original was 40.0

        public VisionOpenCV(HardwareMap hardwareMap){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), this);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
            satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

            if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
            } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
            }
            return CameraSelectedAroundMid.NONE;
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
            submat = input.submat(rect);
            Scalar color = Core.mean(submat);
            return color.val[1];
        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);

            selectionAroundMid = (CameraSelectedAroundMid) userContext;
            switch (selectionAroundMid) {
                case LEFT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
                case RIGHT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
            }
        }

        public void getSelection() {
            if (startPosition == START_POSITION.RED_LEFT ||
                    startPosition == START_POSITION.BLUE_LEFT) {
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            }
        }
    }

    public enum CameraSelectedAroundMid {
        NONE,
        LEFT_OF_CAMERA_MID,
        RIGHT_OF_CAMERA_MID
    }
}   // end class