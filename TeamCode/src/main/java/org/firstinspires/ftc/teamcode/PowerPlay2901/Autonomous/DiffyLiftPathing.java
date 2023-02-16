package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Diffy Lift Pathing", group="Iterative Opmode")
public class DiffyLiftPathing extends OpMode {
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    public double initTheta;

    //Don't need to use this unless camera is not facing a cardinal direction in relation to robot
    final double angleOffset = 0;
    final int maxCycles = 2;

    T265Camera.CameraUpdate up;

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.SECONDS);

    //Grants access to the T265 coordinates and current rotational angle
    Translation2d translation;
    Rotation2d rotation;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime();

    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    //Constants for odometry
    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = 8.375; //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 7.25;
    public static final double inchPerTick = wheelCircumference / encoderTicksPerWheelRev;
    public static final double encoderTicksPerInch = encoderTicksPerWheelRev/wheelCircumference;
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
    double kd = 0.01;
    final double max_i = 1;

    double dx = 0;
    double dy = 0;

    double xTolerance = 3;
    double yTolerance = 3;

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


    public enum AutoState {
        MOVE_FORWARD,
        REVERSE,
        TURN_45,
        LIFT_SLIDES,
        INCH_FORWARD,
        EXTEND_PASSTHROUGH,
        DELIVER,
        RETRACT_SLIDES,
        EXTEND_SLIDES,
        INCH_BACK,
        TURN_452,
        MOVE_BACK,
        RETRACT_CLAW,
        RETRACT_SLIDES_2,
        MOVE_FORWARD2,
        TURN_N45,
        PARK,
        FINAL_TURN
    }
    AutoState autoState;

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

    //Adjusted position for camera's offset from robot's center
    double offsetX;
    double offsetY;
    //Position based on the average readings of the T265 and the odometry wheels
    double averagedX;
    double averagedY;

    double targetAngle2 = 0;

    int liftTarget = 0;
    double feedForward = .3;

    boolean dropping = false;
    boolean timer = false;
    double timerTime = 0;
    boolean liftEngage = false;


    //XYhVector stores the odometry's x, y, and angle values (accessed with pos.x, pos.y, or pos.h)
    public XYhVector START_POS = new XYhVector(-cameraXOffset, -cameraYOffset, 0);
    public XYhVector pos = new XYhVector(START_POS);

    //Ignore for now, use later for parking locations using camera
    public int parking = -1;
    int cycle = 0;

    double leftTarget;
    double rightTarget;

    boolean firstRound = true;
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, true);
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        initTheta = slamra.getLastReceivedCameraUpdate().pose.getRotation().getDegrees() + angleOffset;
        currentTime = time.time(TimeUnit.MILLISECONDS);
        previousTime = currentTime;
        previousError = currentError;
        improvedGamepad = new ImprovedGamepad(gamepad1, impTime, "GP");
        improvedGamepad2 = new ImprovedGamepad(gamepad2, impTime, "GP2");


        //Sets the target position to offsets to prevent initial movement upon starting
        positionX = -cameraXOffset;
        positionY = -cameraYOffset;
        autoState = AutoState.MOVE_FORWARD;

        leftTarget = robot.odoLeft.getCurrentPosition();
        robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        //julia circle vision
        //ElapsedTime stopwatch = new ElapsedTime();
        //double seconds = stopwatch.seconds();
        //pipeline = new ObjectDetectionPipeline(this.telemetry);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        improvedGamepad.update();
        improvedGamepad2.update();
        telemetry.addData("Auto State", autoState);
//        telemetry.addData("x error", dx);
//        telemetry.addData("y error", dy);
//        telemetry.addData("is turning", isTurning);
//        telemetry.addData("is moving", isMoving);
//        telemetry.addData("target angle", targetAngle);
//        telemetry.addData("current angle", robot.getAngle());
//        telemetry.addData("turn power", turnPower);
//        telemetry.addData("parking zone", parking);
//        telemetry.addData("left encoders", robot.odoLeft.getCurrentPosition());
//        telemetry.addData("left target", leftTarget);

        up = slamra.getLastReceivedCameraUpdate();
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();
        if (up == null) return;

        double x1 = translation.getX(), y1 = translation.getY();
        double fieldTheta = rotation.getDegrees() + angleOffset - initTheta;
        //Adjusts for camera initial positional offset
        offsetX = (x1 - (cameraXOffset * Math.cos(rotation.getRadians())));
        offsetY = (y1 - (cameraYOffset * Math.cos(rotation.getRadians())));
        //Adjusts for camera initial angle offset
        double adjustX = ((offsetX * Math.cos(angleOffset)) - (offsetY * Math.sin(angleOffset)));
        double adjustY = ((offsetX * Math.sin(angleOffset)) + (offsetY * Math.cos(angleOffset)));

        //Average of camera and odometry
        averagedX = ((offsetX*0) + (pos.x*1));
        averagedY = ((offsetY*0) + (pos.y*1));

        //Changes target Position
       /*if (improvedGamepad.dpad_right.isInitialPress()) {
           move(24, 0);
       } else if (improvedGamepad.dpad_left.isInitialPress()) {
           move(-24, 0);
       } else if (improvedGamepad.dpad_up.isInitialPress()) {
           move(0, 72);
       } else if (improvedGamepad.dpad_down.isInitialPress()) {
           move(0, -72);
       }
       //Changes target angle
       if(improvedGamepad.a.isInitialPress()) {
           liftTarget = 415;
       } else if(improvedGamepad.b.getValue()){
           targetAngle = 90;
       } else if(improvedGamepad.y.getValue()){
           targetAngle = 180;
       } else if(improvedGamepad.x.getValue()){
           targetAngle = -90;
       }*/

        if(robot.distanceSensor.getDistance(DistanceUnit.INCH) < 2.4 || robot.leftTouch.isPressed() || robot.rightTouch.isPressed()){
            robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }

        if(robot.pipeLine.winner == -1){}
        else if(firstRound) {
            moveInchesForward(52);
            firstRound = false;
            parking = robot.pipeLine.winner;
        }else if(autoState == AutoState.MOVE_FORWARD){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.DELIVER;
                telemetry.addData("Auto State", autoState);
                timer = true;
                runtime.reset();
                timerTime = 1500;
            }
        }else if(autoState == AutoState.DELIVER){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.TURN_45;
                telemetry.addData("Auto State", autoState);
                turnTo(45);
                timer = true;
                runtime.reset();
                timerTime = 1100;
                liftEngage = true;
            }
        }else if(autoState == AutoState.TURN_45){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.LIFT_SLIDES;
                liftTarget = 815;
                isLifting = true;
                timer = true;
                runtime.reset();
                timerTime = 2000;
                liftEngage = true;
            }
        }else if(autoState == AutoState.LIFT_SLIDES){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.INCH_FORWARD;
                xTolerance = 1;
                yTolerance = 1;
                moveInchesForward(10);
            }
        }else if(autoState == AutoState.INCH_FORWARD){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.INCH_BACK;
                xTolerance = 1;
                yTolerance = 1;
                moveInchesForward(-12);
            }
        }else if(autoState == AutoState.INCH_BACK){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.TURN_452;
                telemetry.addData("Auto State", autoState);
                turnTo(-90);
                timer = true;
                runtime.reset();
                timerTime = 1100;
//                timer = true;
//                runtime.reset();
//                timerTime = 1500;
            }
        }else if(autoState == AutoState.TURN_452){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.MOVE_BACK;
                moveInchesForward(26);
                robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            }
        }else if(autoState == AutoState.MOVE_BACK){
            if((!isTurning && !isMoving && !isLifting)) {
                if(cycle == maxCycles || matchTimer.seconds()>22) {
                    autoState = AutoState.PARK;
                } else {
                    autoState = AutoState.MOVE_FORWARD2;
                    moveInchesForward(-26);
                    cycle++;
                }
            }else if(robot.rightTouch.isPressed() || robot.leftTouch.isPressed()){
                if(cycle == maxCycles || matchTimer.seconds()>22) {
                    autoState = AutoState.PARK;
                } else {
                    autoState = AutoState.MOVE_FORWARD2;
                    moveInchesForward(-26);
                    cycle++;
                }
            }
        }else if(autoState == AutoState.MOVE_FORWARD2){
            if(!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.MOVE_FORWARD;
            }
        }else if(autoState == AutoState.PARK){
            if (!isTurning && !isMoving && !isLifting) {
                autoState = AutoState.FINAL_TURN;
                telemetry.addData("Auto State", autoState);

                if(parking == 1){
                    xTolerance = 1;
                    moveInchesForward(-48);
                    timer = true;
                    timerTime = 5000;
                    runtime.reset();
                } else if(parking == 2){
                    xTolerance = 1;
                    moveInchesForward(-24);
                    timer = true;
                    timerTime = 5000;
                    runtime.reset();
                } else if(parking == 0){
                    xTolerance = 1;
                    moveInchesForward(-4);
                    timer = true;
                    timerTime = 5000;
                    runtime.reset();
                }
            }
        }
       /*else if(autoState == AutoState.TURN_452) {
           if (!isTurning && !isMoving) {
               autoState = AutoState.LIFT_SLIDES;
               liftTarget = 815;
               isMoving = true;
           }
       }else if(autoState == AutoState.LIFT_SLIDES){
           if (!isTurning && !isMoving) {
               autoState = AutoState.MOVE_BACK;
               move(26, 0);
           }
       }else if(autoState == AutoState.TURN_45){
           if(!isTurning && !isMoving) {
               autoState = AutoState.TURN_452;
               isTurning = true;
               targetAngle = 90;

           }
       }else if(autoState == AutoState.MOVE_BACK){
           if(!isTurning && !isMoving) {
               autoState = AutoState.MOVE_FORWARD2;
               move(-24, 0);
           }
       } *//*else if(autoState == AutoState.MOVE_FORWARD2){
           if(!isTurning && !isMoving) {
               autoState = AutoState.PARK;
               isTurning = true;
               if(parking == 1) {
                   moveTo(48, -24);
               }else if(parking == 2){
                   moveTo(48, 0);
               } else if(parking == 3){
                   moveTo(48, 24);
               }

           }
       } else if(autoState == AutoState.PARK){
           if(!isTurning && !isMoving) {
               autoState = AutoState.FINAL_TURN;
               isTurning = true;
               targetAngle = 45;
           }
       }*/

        //updates odometry
        //odometry();

        currentError = (leftTarget*inchPerTick - robot.odoLeft.getCurrentPosition()*inchPerTick);

        //Movement PID code
        if (!isTurning && isMoving && (Math.abs(((leftTarget) - (robot.odoLeft.getCurrentPosition()))) > 0)) {
            //If pods are moving perpendicular change this value by +90 or -90
            //If pods are moving opposite direction change this value by +180/-180

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
//            telemetry.addData("angle to Traget", angleToTarget);
//            telemetry.addData("atan2 ", Math.atan2(dy, dx));
//            telemetry.addData("x error", dx);
//            telemetry.addData("y error", dy);


        } else {
            outputLeft = 0;
        }

        outputLeft *= -1;
        //pos.h change (from rotation.toDegrees)



//        telemetry.addData("position x", positionX);
//        telemetry.addData("position y", positionY);
//        telemetry.addData("camera x", averagedX);
//        telemetry.addData("camera y", averagedY);
        telemetry.addData("right touch", robot.rightTouch.isPressed());
        telemetry.addData("left touch", robot.leftTouch.isPressed());
        telemetry.addData("error angle", targetAngle - robot.getAngle());
        telemetry.addData("Distance sensor", robot.distanceSensor.getDistance(DistanceUnit.INCH));

        //Creates dead zone radius larger than target

        double error = AngleUnit.normalizeDegrees(angleToTarget - robot.getAngle());
        //turnPower = -turnPID(error);
        //outputLeft *= Math.cos(Math.toRadians(error));
        if(outputLeft > 1){
            outputLeft = 1;
        } else if(outputLeft < -1){
            outputLeft = -1;
        }
        //if(error > 10 && error < 170 || error < -10 && error > -170){
        //outputLeft = 0;
        //}

        //pos.h change
        outputRight = outputLeft;
        outputRight += turnPower;
        outputLeft -= turnPower;
        leftTurnPower = leftPodTurn(0);
        rightTurnPower = rightPodTurn(0);
        if(liftEngage) {
            runLift(liftTarget, false);
        }

        if(isTurning) {
            //telemetry.addData("target angle", targetAngle);
            turnPower = turnPID(targetAngle - robot.getAngle());
            outputRight = turnPower;
            outputLeft = -turnPower;
            //telemetry.addData("turn power", turnPower);
            if (Math.abs(AngleUnit.normalizeDegrees(targetAngle - robot.getAngle())) < 3 && Math.abs(dTurn) < 10) {
                isTurning = false;
            }
        }

        if(timer && runtime.milliseconds() > timerTime){
            isLifting = false;
            isMoving = false;
            isTurning = false;
            timer = false;
        }

        if (!isTurning && isMoving && (Math.abs(leftTarget*inchPerTick - robot.odoLeft.getCurrentPosition()*inchPerTick)) < yTolerance) {
            outputLeft = 0;
            outputRight = 0;
            leftTurnPower = 0;
            rightTurnPower = 0;
            robot.leftOne.setVelocity(0);
            robot.leftTwo.setVelocity(0);
            robot.rightOne.setVelocity(0);
            robot.rightTwo.setVelocity(0);
            isMoving = false;
        } else if(!isTurning && !isMoving){
            outputLeft = 0;
            outputRight = 0;
            leftTurnPower = 0;
            rightTurnPower = 0;
            robot.leftOne.setVelocity(0);
            robot.leftTwo.setVelocity(0);
            robot.rightOne.setVelocity(0);
            robot.rightTwo.setVelocity(0);
        } else if(isMoving){
            robot.leftOne.setVelocity(((outputLeft-(2.7*Math.toRadians(targetAngle-robot.getAngle())))/speedMod+leftTurnPower)*2500);
            robot.leftTwo.setVelocity(((outputLeft-(2.7*Math.toRadians(targetAngle-robot.getAngle())))/speedMod-leftTurnPower)*2500);
            robot.rightOne.setVelocity(((outputRight+(2.7*Math.toRadians(targetAngle-robot.getAngle())))/speedMod+rightTurnPower)*2500);
            robot.rightTwo.setVelocity(((outputRight+(2.7*Math.toRadians(targetAngle-robot.getAngle())))/speedMod-rightTurnPower)*2500);
        } else {
            robot.leftOne.setVelocity((outputLeft/speedMod+leftTurnPower)*2500);
            robot.leftTwo.setVelocity((outputLeft/speedMod-leftTurnPower)*2500);
            robot.rightOne.setVelocity((outputRight/speedMod+rightTurnPower)*2500);
            robot.rightTwo.setVelocity((outputRight/speedMod-rightTurnPower)*2500);
        }
//        telemetry.addData("leftTurnPower", leftTurnPower);
//        telemetry.addData("rightTurnPower", rightTurnPower);
//        telemetry.addData("outputLeft", outputLeft);
//        telemetry.addData("outputRight", outputRight);
//        telemetry.addData("Distance to Target x", (positionX - averagedX));
//        telemetry.addData("Distance to Target y", (positionY - averagedY));

       /*if(isTurning) {
           leftTurnPower = leftPodTurn(0);
           rightTurnPower = rightPodTurn(0);
           outputLeft = turnToAngle(targetAngle);
           outputRight = -outputLeft;
           if (Math.abs(outputLeft) < 0.01) {
               isTurning = false;
           }
       }*/
        //runLift(liftTarget, dropping);

        if(improvedGamepad2.dpad_up.isInitialPress()){
            turnKp += 0.01;
        } else if(improvedGamepad2.dpad_down.isInitialPress()){
            turnKp -= 0.01;
        } else if(improvedGamepad2.dpad_left.isInitialPress()){
            turnKi -= 0.01;
        } else if(improvedGamepad2.dpad_right.isInitialPress()){
            turnKi += 0.01;
        } else if(improvedGamepad2.y.isInitialPress()){
            turnKd += 0.01;
        } else if(improvedGamepad2.a.isInitialPress()){
            turnKd -= 0.01;
        }

       /*telemetry.addData("isturning", isTurning);
       telemetry.addData("isMoving", isMoving);
       telemetry.addData("Auto State", autoState);
       telemetry.addData("output", outputLeft);
       telemetry.addData("output right", outputRight);
       telemetry.addData("x1", String.format("%.2f", x1));
       telemetry.addData("y1", String.format("%.2f", y1));
       telemetry.addData("Angle to Target", angleToTarget + "°");*/
    }

    @Override
    public void stop() {
        slamra.stop();
    }

    //enter (x, y) coordinates to move robot by
    public void move(double x, double y) {
        positionX = x + (averagedX);
        positionY = y + (averagedY);
        isMoving = true;
    }

    //enter (x, y) coordinates to move robot to
    public void moveTo(double x, double y) {
        positionX = x;
        positionY = y;
        isMoving = true;
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
        if (!improvedGamepad.start.getValue() && (error >= 90 || error <= -90)) {
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
        if (!improvedGamepad.start.getValue() && (error >= 90 || error <= -90)) {
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

//        robot.liftOne.setPower((liftPower - feedForward) * scaleFactor);
//        robot.liftTwo.setPower((liftPower - feedForward) * scaleFactor);
    }

    private ElapsedTime runtimeTurn = new ElapsedTime();
    double pTurn = 0;
    double iTurn = 0;
    double dTurn = 0;
    double ktp = 1.47;
    double kti = 0.02;
    double ktd = 0.42;

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

    public void moveInchesForward(double inches){
        robot.odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTarget = robot.odoLeft.getCurrentPosition() + inches*encoderTicksPerInch;
        rightTarget = robot.odoRight.getCurrentPosition() + inches*encoderTicksPerInch;
        isMoving = true;
    }


    public void turnTo(double degrees){
        isTurning = true;
        targetAngle = degrees;
    }

}

