package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "IRA", group = "All")
public class IntelRealsenseFinal extends LinearOpMode {
    private static T265Camera slamra = null;
    public double initTheta;

    final double angleOffset = 0;

    T265Camera.CameraUpdate up;

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.SECONDS);

    Translation2d translation;
    Rotation2d rotation;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime matchTimer = new ElapsedTime();


    //Constants for odometry
    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = 8.375; //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 7.25;
    public static final double inchPerTick = wheelCircumference / encoderTicksPerWheelRev;
    public static final double wheelCircumferenceBack = (2 * Math.PI);
    public static final double backInchPerTick = wheelCircumferenceBack / encoderTicksPerWheelRev;

    //Variables for odometry
    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;
    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;

    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double p;
    double i;
    double d;

    //PID constants for moving
    double kp = 0.07;
    double ki = 0;
    double kd = 0;
    final double max_i = 1;

    double dx = 0;
    double dy = 0;

    double xTolerance = 3;
    double yTolerance = 0.5;

    double addX = 0;
    double addY = 0;

    //positionX and positionY are the target coordinates. (set them to make the robot move)
    double positionX, positionY;
    double targetAngle = 0;

    double cameraXOffset = 6; //lies 6 inches in front of middle
    double cameraYOffset = -3.5; //lies 3.5 inches up from the middle

    double angleToTarget = 0;

    double turnByAngle;

    //Keeps track of current actions of robot for purposes of switching AutoState
    boolean isMoving = false;
    boolean isTurning = false;
    boolean isLifting = false;

    double liftPower = 0;
    double randomInt = 0;

    double outputLeft;
    double outputRight;
    double speedMod = 3;

    double turnPower;
    double leftTurnPower;
    double rightTurnPower;

    ImprovedGamepad improvedGamepad;
    ImprovedGamepad improvedGamepad2;

    double turnAngle;

    //Turning PID constants
    double turnKp = -0.44;
    double turnKi = 0;
    double turnKd = 0;

    double averagedX;
    double averagedY;


    double feedForward = .3;

    public XYhVector START_POS = new XYhVector(-cameraXOffset, -cameraYOffset, 0);
    public XYhVector pos = new XYhVector(START_POS);

    public int parking = -1;

    boolean firstRound = true;
    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, true);

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.start();
        currentTime = time.time(TimeUnit.MILLISECONDS);
        previousTime = currentTime;
        previousError = currentError;
        //Sets the target position to offsets to prevent initial movement upon starting
        positionX = -cameraXOffset;
        positionY = -cameraYOffset;
        up = slamra.getLastReceivedCameraUpdate();
        // We divide by 0.0254 to convert meters to inches
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        while(translation.getX() == 0){
            up = slamra.getLastReceivedCameraUpdate();
            // We divide by 0.0254 to convert meters to inches
            translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        }

        waitForStart();
//        move(-4, 48);
//        turn(45);
        spline(-4, 48, 45);
        resetPods();

        slamra.stop();


    }
    public void turn(double angle){
        timer.reset();
        while(Math.abs(AngleUnit.normalizeDegrees(angle - robot.getAngle())) > 1 && timer.milliseconds() < 2000){
            odometry();
            double turnPower = turnPID(angle - robot.getAngle());
            double outputRight = -turnPower;
            double outputLeft = turnPower;
            robot.leftOne.setVelocity((outputLeft/3)*2500);
            robot.leftTwo.setVelocity((outputLeft/3)*2500);
            robot.rightOne.setVelocity((outputRight/3)*2500);
            robot.rightTwo.setVelocity((outputRight/3)*2500);
        }
        robot.leftOne.setVelocity(0);
        robot.leftTwo.setVelocity(0);
        robot.rightOne.setVelocity(0);
        robot.rightTwo.setVelocity(0);
    }


    public void spline(double x, double y, double angle){
        up = slamra.getLastReceivedCameraUpdate();
        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();
        double offsetX = (translation.getX() - (cameraXOffset * Math.cos(rotation.getRadians())));
        double offsetY = (translation.getY() - (cameraYOffset * Math.cos(rotation.getRadians())));
        odometry();
        positionX = x + (offsetX);
        positionY = y + (offsetY);
        dx = y;
        dy = x;
        currentError = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
        while(currentError > 2) {
            telemetry.addData("distance", currentError);
            telemetry.addData("angle to target", angleToTarget);
            telemetry.update();

            up = slamra.getLastReceivedCameraUpdate();
            // We divide by 0.0254 to convert meters to inches
            translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            rotation = up.pose.getRotation();
            offsetX = (translation.getX() - (cameraXOffset * Math.cos(rotation.getRadians())));
            offsetY = (translation.getY() - (cameraYOffset * Math.cos(rotation.getRadians())));
            dx = positionX - offsetX;
            dy = positionY - offsetY;
            currentError = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
            angleToTarget = -Math.toDegrees(Math.atan2(-dx, dy));

            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * currentError;
            i += ki * (currentError * (currentTime - previousTime));

            if (i > max_i) {
                i = max_i;
            } else if (i < -max_i) {
                i = -max_i;
            }

            d = kd * (currentError - previousError) / (currentTime - previousTime);

            outputLeft = p + i + d;

            previousError = currentError;
            previousTime = currentTime;
            if (outputLeft > 1) {
                outputLeft = 1;
            } else if (outputLeft < -1) {
                outputLeft = -1;
            }
            outputLeft *= -1;
            outputRight = outputLeft;



            robot.leftOne.setVelocity(((outputLeft-turnPID(angle-robot.getAngle())/4)/3+leftPodTurn(angleToTarget))*2500);
            robot.leftTwo.setVelocity(((outputLeft-turnPID(angle-robot.getAngle())/4)/3-leftPodTurn(angleToTarget))*2500);
            robot.rightOne.setVelocity(((outputRight+turnPID(angle-robot.getAngle())/4)/3+rightPodTurn(angleToTarget))*2500);
            robot.rightTwo.setVelocity(((outputRight+turnPID(angle-robot.getAngle())/4)/3-rightPodTurn(angleToTarget))*2500);
        }
        resetPods();
        robot.leftOne.setVelocity(0);
        robot.leftTwo.setVelocity(0);
        robot.rightOne.setVelocity(0);
        robot.rightTwo.setVelocity(0);
    }
    //enter (x, y) coordinates to move robot by
    public void move(double x, double y) {
        up = slamra.getLastReceivedCameraUpdate();
        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();
        double offsetX = (translation.getX() - (cameraXOffset * Math.cos(rotation.getRadians())));
        double offsetY = (translation.getY() - (cameraYOffset * Math.cos(rotation.getRadians())));
        odometry();
        positionX = x + (offsetX);
        positionY = y + (offsetY);
        dx = y;
        dy = x;
        currentError = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
        while(currentError > 2) {
            telemetry.addData("distance", currentError);
            telemetry.addData("angle to target", angleToTarget);
            telemetry.update();
            up = slamra.getLastReceivedCameraUpdate();
            // We divide by 0.0254 to convert meters to inches
            translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            rotation = up.pose.getRotation();
            offsetX = (translation.getX() - (cameraXOffset * Math.cos(rotation.getRadians())));
            offsetY = (translation.getY() - (cameraYOffset * Math.cos(rotation.getRadians())));
            dx = positionX - offsetX;
            dy = positionY - offsetY;
            currentError = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
            angleToTarget = -Math.toDegrees(Math.atan2(-dx, dy));

            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * currentError;
            i += ki * (currentError * (currentTime - previousTime));

            if (i > max_i) {
                i = max_i;
            } else if (i < -max_i) {
                i = -max_i;
            }

            d = kd * (currentError - previousError) / (currentTime - previousTime);

            outputLeft = p + i + d;

            previousError = currentError;
            previousTime = currentTime;
            if (outputLeft > 1) {
                outputLeft = 1;
            } else if (outputLeft < -1) {
                outputLeft = -1;
            }
            outputLeft *= -1;
            outputRight = outputLeft;

            robot.leftOne.setVelocity((outputLeft/3+leftPodTurn(angleToTarget))*2500);
            robot.leftTwo.setVelocity((outputLeft/3-leftPodTurn(angleToTarget))*2500);
            robot.rightOne.setVelocity((outputRight/3+rightPodTurn(angleToTarget))*2500);
            robot.rightTwo.setVelocity((outputRight/3-rightPodTurn(angleToTarget))*2500);
        }
        resetPods();
        robot.leftOne.setVelocity(0);
        robot.leftTwo.setVelocity(0);
        robot.rightOne.setVelocity(0);
        robot.rightTwo.setVelocity(0);
    }

    public void resetPods(){
        runtime.reset();
        while(opModeIsActive()) {
            double leftPodPower = leftPodTurn(0);
            double rightPodPower = rightPodTurn(0);
            robot.leftOne.setVelocity(leftPodPower * 2500);
            robot.leftTwo.setVelocity(-leftPodPower * 2500);
            robot.rightOne.setVelocity(rightPodPower * 2500);
            robot.rightTwo.setVelocity(-rightPodPower * 2500);
            if(runtime.milliseconds() > 500) break;
        }
        robot.leftOne.setVelocity(0);
        robot.leftTwo.setVelocity(0);
        robot.rightOne.setVelocity(0);
        robot.rightTwo.setVelocity(0);
    }


    //Left Pod PID
    //private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    double kpPod = 1.2;
    double kiPod = 0;
    double kdPod = 0;

    public double leftPodTurn(double angle) {
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition()) / 8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if ((error >= 90 || error <= -90)) {
            error = AngleUnit.normalizeDegrees(error - 180);
            outputLeft = -outputLeft;
        }
        /*double secs = runtimePodLeft.seconds();
        runtimePodLeft.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);*/
        pAngleLeft = error;
        double total = (kpPod * pAngleLeft + kiPod * iAngleLeft + kdPod * dAngleLeft) / 100;
        if (total > 1) {
            iAngleLeft = 0;
            total = 1;
        }
        if (total < -1) {
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    //Right Pod PID
    //private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle) {
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition()) / 8.95;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if ((error >= 90 || error <= -90)) {
            error = AngleUnit.normalizeDegrees(error - 180);
            outputRight = -outputRight;
        }
        /*double secs = runtimePodRight.seconds();
        runtimePodRight.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);*/
        pAngleRight = error;
        double total = (kpPod * pAngleRight + kiPod * iAngleRight + kdPod * dAngleRight) / 100;
        if (total > 1) {
            iAngleRight = 0;
            total = 1;
        }
        if (total < -1) {
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }

    //Enter angle for robot to turn by, robot oriented
    /*public double turnToAngle(double turnAngle){
        double targetAngle = turnAngle;
        ElapsedTime runtime = new ElapsedTime();
        double p = 0;
        double i = 0;
        double d = 0;
        double error = turnAngle;

        while(!(error < 1 && error > -1)){
            error = AngleUnit.normalizeDegrees(targetAngle - Math.toDegrees(robot.getAngle()));
            double secs = runtime.seconds();
            runtime.reset();
            d = (error - p) / secs;
            i = i + (error * secs);
            p = error;
            double total = (turnKp* p + turnKi* i + turnKd* d)/100;
            if(total > 1){
                i = 0;
                total = 1;
            }
            if(total < -1){
                i = 0;
                total = -1;
            }
            return total;
        }

        return 0;
    }*/

    //Gets coordinates of robot using 3 dead wheels with encoders
    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldBackPosition = currentBackPosition;

        currentRightPosition = robot.odoRight.getCurrentPosition();
        currentLeftPosition = robot.odoLeft.getCurrentPosition();
        currentBackPosition = robot.liftTwo.getCurrentPosition();

        int dn1 = currentRightPosition - oldRightPosition;
        int dn2 = currentLeftPosition - oldLeftPosition;
        int dn3 = currentBackPosition - oldBackPosition;

        dn2 = -dn2;

        double dtheta = ((dn2 - dn1) / leftRightDistance) * inchPerTick;
        double dx = ((dn1 + dn2) / 2) * inchPerTick;
        double dy = ((backInchPerTick * dn3) - (inchPerTick * (dn2 - dn1) * midpointBackDistance) / leftRightDistance);

        double theta = pos.h + (dtheta / 2.0);
        pos.y += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.x += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }

    public void runLift(int target, boolean drop){
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;

        if(drop){
            liftPower = liftPower(target - 65);
            feedForward = 0;
            liftI = 0;
        } else {
            liftPower = liftPower(target);
            feedForward = .3;
            //telemetry.addData("Lift Power", liftPower);
        }

        robot.liftOne.setPower((liftPower - feedForward) * scaleFactor);
        robot.liftTwo.setPower((liftPower - feedForward) * scaleFactor);
    }

    private ElapsedTime runtimeTurn = new ElapsedTime();
    double pTurn = 0;
    double iTurn = 0;
    double dTurn = 0;
    double ktp = 1.25;
    double kti = 0;
    double ktd = 0.3;

    public double turnPID(double error){
        double secs = runtimeTurn.seconds();
        runtimeTurn.reset();
        dTurn = (error - pTurn) / secs;
        iTurn = iTurn + (error * secs);
        pTurn = error;
        double total = (ktp* pTurn + kti* iTurn + ktd*dTurn)/100;
        if(total > 1){
            iTurn = 0;
            total = 1;
        }
        if(total < -1){
            iTurn = 0;
            total = -1;
        }
        return total;
    }

    double klp = 0.55;
    double kli = 0;
    double kld = 0.015;

    public ElapsedTime runtimeLift = new ElapsedTime();
    double liftP = 0;
    double liftI = 0;
    double liftD = 0;

    public double liftPower(int target){
        int error = robot.liftOne.getCurrentPosition() - target;
        telemetry.addData("error", error);
        double secs = runtimeLift.seconds();
        runtimeLift.reset();
        liftD = (error - liftP) / secs;
        liftI = liftI + (error * secs);
        liftP = error;
        double total = (klp* liftP + kli* liftI + kld* liftD)/100;
        if(total > .65){
            liftI = 0;
            total = .65;
        }
        if(total < -1){
            liftI = 0;
            total = -1;
        }
        if(Math.abs(error) < 4){
            isLifting = false;
        }
        return total;
    }
}
