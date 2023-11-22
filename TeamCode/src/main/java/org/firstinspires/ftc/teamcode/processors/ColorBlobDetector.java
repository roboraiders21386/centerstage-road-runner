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
    /*
    public enum Location {

        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;
*/
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    static final Rect LEFT_RECT = new Rect(1,1,426,719);
    static final Rect MIDDLE_RECT = new Rect(427, 1, 426, 719);

    static final Rect RIGHT_RECT = new Rect(853, 1, 427, 719);

    Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public ColorBlobDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        double leftavgfin;
        double middleavgfin;
        double rightavgfin;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("Pipeline running");
        input.copyTo(outputMat);
        Imgproc.rectangle(outputMat, LEFT_RECT, rectColor, 2);
        Imgproc.rectangle(outputMat, MIDDLE_RECT, rectColor, 2);
        Imgproc.rectangle(outputMat, RIGHT_RECT, rectColor, 2);


        Mat rightMat = mat.submat(RIGHT_RECT);
        Mat middleMat = mat.submat(MIDDLE_RECT);
        Mat leftMat = mat.submat(LEFT_RECT);

        //Core.extractChannel(leftCrop, leftCrop, 1);
        //Core.extractChannel(middleCrop, middleCrop, 1);
        //Core.extractChannel(rightCrop, rightCrop, 1);

        double leftValue = Core.sumElems(leftMat).val[0] / LEFT_RECT.area() / 255;
        double rightValue = Core.sumElems(rightMat).val[0] / RIGHT_RECT.area() / 255;
        double middleValue = Core.sumElems(middleMat).val[0]/MIDDLE_RECT.area()/255;

        leftMat.release();
        rightMat.release();
        middleMat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(leftMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(rightMat).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middleMat).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 100) + "%");

        if ((leftValue > PERCENT_COLOR_THRESHOLD) && (middleValue < PERCENT_COLOR_THRESHOLD)){
            telemetry.addLine("Left");
            //location = Location.LEFT;
        } else if ((middleValue > PERCENT_COLOR_THRESHOLD) && (rightValue < PERCENT_COLOR_THRESHOLD)){
            telemetry.addLine("Middle");
            //location = Location.MIDDLE;
        }else{
            telemetry.addLine("Right");
            //location = Location.RIGHT;
        };

        /*
        Scalar leftavg = Core.mean(leftCrop);
        Scalar middleavg = Core.mean(middleCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgfin = leftavg.val[0];
        middleavgfin = middleavg.val[0];
        rightavgfin = rightavg.val[0];

        if ((leftavgfin > middleavgfin) && (leftavgfin > rightavgfin)) {
            telemetry.addLine("Left");
            location = Location.LEFT;
        } else if ((middleavgfin > leftavgfin) && (middleavgfin > rightavgfin)) {
            telemetry.addLine("Middle");
            location = Location.MIDDLE;
        } else{
            telemetry.addLine("Right");
            location = Location.RIGHT;
        };
        */

        return (outputMat);
        /*
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;

         */
    }
/*
    public Location getLocation() {
        return location;
    }

 */
}