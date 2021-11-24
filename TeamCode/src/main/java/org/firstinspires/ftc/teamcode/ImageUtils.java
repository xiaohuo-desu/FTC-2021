package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class ImageUtils {
    private static final int BLACK = 0;
    private static final int WHITE = 255;

    private Mat mat;

    /**
     * 空参构造函数
     */
    public ImageUtils() {

    }

    /**
     * 通过图像路径创建一个mat矩阵
     *

     *            图像路径
     */
    public void getMat(Mat MAT){
        mat=MAT;
    }

    public void ImageUtils(Mat mat) {
        this.mat = mat;
    }

    /**
     * 加载图片
     *
     * @param imgFilePath
     */
    public void loadImg(String imgFilePath) {
        mat = Imgcodecs.imread(imgFilePath);
    }

    /**
     * 获取图片高度的函数
     *
     * @return
     */
    public int getHeight() {
        return mat.rows();
    }

    /**
     * 获取图片宽度的函数
     *
     * @return
     */
    public int getWidth() {
        return mat.cols();
    }

    /**
     * 获取图片像素点的函数
     *
     * @param y
     * @param x
     * @return
     */
    public int getPixel(int y, int x) {
        // 我们处理的是单通道灰度图
        return (int) mat.get(y, x)[0];
    }

    /**
     * 设置图片像素点的函数
     *
     * @param y
     * @param x
     * @param color
     */
    public void setPixel(int y, int x, int color) {
        // 我们处理的是单通道灰度图
        mat.put(y, x, color);
    }

    /**
     * 保存图片的函数
     *
     * @param filename
     * @return
     */
    public boolean saveImg(String filename) {
        return Imgcodecs.imwrite(filename, mat);
    }
}


