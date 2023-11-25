package org.firstinspires.ftc.teamcode.processors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorBlobDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    Mat outputMat = new Mat();

    public enum Location {

        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;

    /*
    static final Rect LEFT_RECT = new Rect(1,1,426,719);
    static final Rect MIDDLE_RECT = new Rect(427, 1, 426, 719);
    static final Rect RIGHT_RECT = new Rect(853, 1, 427, 719);
*/
    static final Rect LEFT_RECT = new Rect(0, 180, 115, 115);
    static final Rect MIDDLE_RECT = new Rect(316, 180, 115, 115);
    static final Rect RIGHT_RECT = new Rect(660, 180, 115, 115);

    //For now check for blue
    Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public ColorBlobDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("Pipeline running");
        input.copyTo(outputMat);
        Imgproc.rectangle(outputMat, LEFT_RECT, rectColor, 2);
        Imgproc.rectangle(outputMat, MIDDLE_RECT, rectColor, 2);
        Imgproc.rectangle(outputMat, RIGHT_RECT, rectColor, 2);

        Mat rightMat = mat.submat(RIGHT_RECT);
        Mat middleMat = mat.submat(MIDDLE_RECT);
        Mat leftMat = mat.submat(LEFT_RECT);

        double leftValue = Core.sumElems(leftMat).val[0] / LEFT_RECT.area() / 255;
        double rightValue = Core.sumElems(rightMat).val[0] / RIGHT_RECT.area() / 255;
        double middleValue = Core.sumElems(middleMat).val[0]/MIDDLE_RECT.area()/255;

        telemetry.addData("Left raw value", (int) Core.sumElems(leftMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(rightMat).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middleMat).val[0]);


        //Averaging the colors in the zones
        Scalar avgColorLeft = Core.mean(leftMat);
        Scalar avgColorRight = Core.mean(rightMat);
        Scalar avgColorMiddle = Core.mean(middleMat);

        telemetry.addData("avgColorLeft", avgColorLeft);
        telemetry.addData("avgColorRight", avgColorRight);
        telemetry.addData("avgColorMiddle", avgColorMiddle);

        //Putting averaged colors on zones (we can see on camera now)
        leftMat.setTo(avgColorLeft);
        rightMat.setTo(avgColorRight);
        middleMat.setTo(avgColorMiddle);

        leftMat.release();
        rightMat.release();
        middleMat.release();


        telemetry.addData("leftValue", leftValue);
        telemetry.addData("rightValue", rightValue);
        telemetry.addData("middleValue", middleValue);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 100) + "%");

        if ((leftValue > PERCENT_COLOR_THRESHOLD) && (middleValue < PERCENT_COLOR_THRESHOLD)){
            telemetry.addData("Pipeline position: ", "Left");
            location = Location.LEFT;
        } else if ((middleValue > PERCENT_COLOR_THRESHOLD) && (rightValue < PERCENT_COLOR_THRESHOLD)){
            telemetry.addData("Pipeline position: ", "Middle");
            location = Location.MIDDLE;
        }else{
            telemetry.addData("Pipeline position: ", "Right");
            location = Location.RIGHT;
        };

       telemetry.update();

        return (outputMat);

    } // end processMat

    public Location getLocation() {
        return location;
    }


}