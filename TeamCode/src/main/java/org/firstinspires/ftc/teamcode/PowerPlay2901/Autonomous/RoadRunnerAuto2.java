package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;

@Autonomous(name = "ROAD RUNNER AUTO 2!!!", group = "010")
public class RoadRunnerAuto2 extends LinearOpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double liftFeedForward = -.0006;
    int parking = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot.init(hardwareMap, telemetry, true);
        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        Trajectory goToPole = robot.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(32, 4.5), Math.toRadians(45)).build();
        Trajectory coneToPole = robot.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(20, -9), Math.toRadians(-60)).build();
        Trajectory poleToCone = robot.trajectoryBuilder(new Pose2d(), true).splineTo(new Vector2d(-13, -13), Math.toRadians(70)).build();
        waitForStart();
        matchTimer.reset();

        while(true){
            if(this.robot.pipeLine.winner != -1){
                parking = this.robot.pipeLine.winner;
                break;
            }
        }
        robot.followTrajectory(goToPole);
        resetPods();
        for(int i = 0; i < 6; i++) {
            robot.followTrajectory(poleToCone);
            resetPods();
            robot.followTrajectory(coneToPole);
            resetPods();
            if(matchTimer.seconds() > 24){
                robot.followTrajectory(poleToCone);
                resetPods();
                park();
                break;
            }
        }
    }

    public void park(){
        if(parking == 1){
            move(48);
        }else if(parking == 2){
            move(24);
        }else if(parking == 0){
        }
    }

    public void slightMove(double inches){
        timer.reset();
        while(timer.milliseconds() < Math.abs((inches*38.96))){
            if(inches > 0) {
                this.robot.leftOne.setVelocity(-2500 / 3.0);
                this.robot.leftTwo.setVelocity(-2500 / 3.0);
                this.robot.rightOne.setVelocity(-2500 / 3.0);
                this.robot.rightTwo.setVelocity(-2500 / 3.0);
            } else if(inches < 0){
                this.robot.leftOne.setVelocity((1-(2.7*Math.toRadians(90-robot.getAngle()))) * 2500 / 3.0);
                this.robot.leftTwo.setVelocity((1-(2.7*Math.toRadians(90-robot.getAngle()))) * 2500 / 3.0);
                this.robot.rightOne.setVelocity((1+(2.7*Math.toRadians(90-robot.getAngle()))) * 2500 / 3.0);
                this.robot.rightTwo.setVelocity((1+(2.7*Math.toRadians(90-robot.getAngle()))) * 2500 / 3.0);
            }
        }
        this.robot.leftOne.setVelocity(0);
        this.robot.leftTwo.setVelocity(0);
        this.robot.rightOne.setVelocity(0);
        this.robot.rightTwo.setVelocity(0);
    }
    public void move(double inches){
        timer.reset();
        while(timer.milliseconds() < Math.abs((inches*38.96))){
            if(inches > 0) {
                this.robot.leftOne.setVelocity(-2500 / 3.0);
                this.robot.leftTwo.setVelocity(-2500 / 3.0);
                this.robot.rightOne.setVelocity(-2500 / 3.0);
                this.robot.rightTwo.setVelocity(-2500 / 3.0);
            } else if(inches < 0){
                this.robot.leftOne.setVelocity(2500 / 3.0);
                this.robot.leftTwo.setVelocity(2500 / 3.0);
                this.robot.rightOne.setVelocity(2500 / 3.0);
                this.robot.rightTwo.setVelocity(2500 / 3.0);
            }
        }
        this.robot.leftOne.setVelocity(0);
        this.robot.leftTwo.setVelocity(0);
        this.robot.rightOne.setVelocity(0);
        this.robot.rightTwo.setVelocity(0);
    }
    public void turn(double angle){
        timer.reset();
        while(Math.abs(AngleUnit.normalizeDegrees(angle - robot.getAngle())) > 1 && timer.milliseconds() < 1750){
            double turnPower = turnPID(angle - robot.getAngle());
            double outputRight = turnPower;
            double outputLeft = -turnPower;
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

    private ElapsedTime runtimeTurn = new ElapsedTime();
    double pTurn = 0;
    double iTurn = 0;
    double dTurn = 0;
    double ktp = 1.30;
    double kti = 0.00;
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

    public void moveLift(int target){
        runtime.reset();
        while (opModeIsActive()){
            telemetry.addData("current position", robot.liftOne.getCurrentPosition());
            telemetry.addData("target", target);
            double liftPower = liftPower(target);
            telemetry.addData("lift power", liftPower);
            robot.liftOne.setPower(liftPower + liftFeedForward);
            robot.liftTwo.setPower(liftPower + liftFeedForward);
            telemetry.addData("time elapsed", runtime.seconds());
            telemetry.update();
            if((Math.abs(robot.liftOne.getCurrentPosition() - target) < 1 && Math.abs(liftD) < 10) || runtime.seconds() > 3){
                break;
            }
        }
        robot.liftOne.setPower(liftFeedForward);
        robot.liftTwo.setPower(liftFeedForward);
    }

    double klp = 0.7;
    double kli = 0.0005;
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
        return total;
    }

    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = 8.375; //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 7.25;
    public static final double inchPerTick = wheelCircumference / encoderTicksPerWheelRev;
    public static final double wheelCircumferenceBack = (2 * Math.PI);
    public static final double backInchPerTick = wheelCircumferenceBack / encoderTicksPerWheelRev;
    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;
    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;
    double cameraXOffset = 6; //lies 6 inches in front of middle
    double cameraYOffset = -3.5;
    public XYhVector START_POS = new XYhVector(-cameraXOffset, -cameraYOffset, 0);
    public XYhVector pos = new XYhVector(START_POS);
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

}
