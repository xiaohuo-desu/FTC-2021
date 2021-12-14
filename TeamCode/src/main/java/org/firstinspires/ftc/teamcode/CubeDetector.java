package org.firstinspires.ftc.teamcode;

import com.google.ftcresearch.tfod.util.ImageUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CubeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    ImageUtils tool = new ImageUtils();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        UNFOUND
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(110, 330),
            new Point(300, 560));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(560, 350),
            new Point(700, 560));
    static final Rect RIGHT_ROI = new Rect(
            new Point(950, 330),
            new Point(1100, 560));
    static double PERCENT_COLOR_THRESHOLD = 0.3;

    public CubeDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        //tool.getMat(input);
        //contoursRemoveNoise(1);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(6,39,56);
        Scalar highHSV = new Scalar(62, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);



        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMiddle= middleValue> PERCENT_COLOR_THRESHOLD;

        if (stoneMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Skystone Location", "middle");
        }
        else if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }
        else if(stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.UNFOUND;
            telemetry.addData("Skystone Location", "unfound");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}