package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectDetectionPipeline extends OpenCvPipeline {
    Mat lastImage = null;
    private Telemetry telemetry;
    public int winner = -1;
    public int framesProcessed = 0;
    public int NUM_FRAMES = 200;
    public int count1 = 0;
    public int count2 = 0;
    public int count0 = 0;

    int count = 0;
    public ObjectDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        framesProcessed++;
        telemetry.addData("count", count);
        count++;

        if (input == null)
            return null;

        if (lastImage != null) {
            lastImage.release();
        }

        lastImage = new Mat();
        input.copyTo(lastImage);

        //converting the video to a gray scale
        Mat grayImage = new Mat();
        Imgproc.cvtColor(lastImage, grayImage, Imgproc.COLOR_RGB2GRAY);

        //blurring the image
        Mat blurImage = grayImage;
        //Imgproc.medianBlur(grayImage, blurImage, 5);

        telemetry.addData("width", blurImage.width());
        telemetry.addData("height", blurImage.height());
        int x = blurImage.width()/2;
        int y = blurImage.height()/2;

        Rect cropRect = new Rect(x + 30, y + 20, 70, 90);
        Imgproc.rectangle(blurImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat cropImg = new Mat(blurImage, cropRect);

        Mat circleImage = new Mat();
        //detecting hough circles
        //decreasing param2 will have it detect more circles (possibly even too many)
        Imgproc.HoughCircles(cropImg, circleImage, Imgproc.HOUGH_GRADIENT, 1, 4, 100, 30, 5, 25);

        for (int i = 0; i < circleImage.cols(); i++) {
            double[] data = circleImage.get(0, i);
            //drawing circle that has been found
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(cropImg, center, 1, new Scalar(255, 255, 255), 2, 8, 0);
            // circle outline
            int radius = (int) Math.round(data[2]);
            Imgproc.circle(cropImg, center, radius, new Scalar(255, 255, 255), 2, 8, 0);
        }
        if (framesProcessed < 400) {
            if (circleImage.cols() == 1) {
                count1++;
            }
        if (circleImage.cols() == 2) {
            count2++;
        }
        if (circleImage.cols() == 0) {
            count0++;

        }
    }
        telemetry.addData("Count 1:", count1);
        telemetry.addData("Count 2:", count2);
        telemetry.addData("Count 0:",count0);
        if(framesProcessed > 100){
            count0 = 0;
            count1 = 0;
            count2 = 0;
            framesProcessed = 0;
            winner = -1;
        }
        //to make sure that there are actually 2 circles but accommodating to the toggle
        if(framesProcessed > 45) {
            if (count2 > 30 && count2 > count1 && count1 < 45) {
                winner = 2;
            }
            //if the amount of times that there is one is over 15 and it is greater than seeing 2 circles
            if (count1 > 40 && winner != 2) {
                winner = 1;
            }
            //only if 0 circles is greater than the amount of times that it sees 1 and 2 circles
            if (count0 > count1 && count0 > count2 && count0>20 && winner != 2 && winner!= 1) {
                winner = 0;
            }
        }
        telemetry.addData("Winner is ", winner);
        telemetry.addData("Frames:",framesProcessed);

        telemetry.update();
        //return blurImage;*/

        return cropImg;
    }
}

