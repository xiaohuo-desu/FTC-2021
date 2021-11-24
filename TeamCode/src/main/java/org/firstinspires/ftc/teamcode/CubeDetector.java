package org.firstinspires.ftc.teamcode;

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
            new Point(60, 330),
            new Point(240, 460));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(580, 350),
            new Point(710, 460));
    static final Rect RIGHT_ROI = new Rect(
            new Point(1090, 330),
            new Point(1280, 460));
    static double PERCENT_COLOR_THRESHOLD = 0.3;

    public CubeDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        //tool.getMat(input);
        //contoursRemoveNoise(1);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23,70,70);
        Scalar highHSV = new Scalar(34, 255, 255);

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

    /**
     * 连通域降噪
     * @param pArea 默认值为1
     */
    public void contoursRemoveNoise(double pArea) {
        int i, j, color = 1;
        int nWidth = tool.getWidth(), nHeight = tool.getHeight();

        for (i = 0; i < nWidth; ++i) {
            for (j = 0; j < nHeight; ++j) {
                if (tool.getPixel(j, i) == 0) {
                    //用不同颜色填充连接区域中的每个黑色点
                    //floodFill就是把一个点x的所有相邻的点都涂上x点的颜色，一直填充下去，直到这个区域内所有的点都被填充完为止
                    Imgproc.floodFill(mat, new Mat(), new Point(i, j), new Scalar(color));
                    color++;
                }
            }
        }

        //统计不同颜色点的个数
        int[] ColorCount = new int[255];

        for (i = 0; i < nWidth; ++i) {
            for (j = 0; j < nHeight; ++j) {
                if (tool.getPixel(j, i) != 255) {
                    ColorCount[tool.getPixel(j, i) -1]++;
                }
            }
        }

        //去除噪点
        for (i = 0; i < nWidth; ++i) {
            for (j = 0; j < nHeight; ++j) {

                if (ColorCount[tool.getPixel(j, i) -1] <= pArea) {
                    tool.setPixel(j, i, 255);
                }
            }
        }

        for (i = 0; i < nWidth; ++i) {
            for (j = 0; j < nHeight; ++j) {
                if (tool.getPixel(j, i) < 255) {
                    tool.setPixel(j, i, 0);
                }
            }
        }

    }

}