package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Autonomous(name = "ROAD RUNNER AUTO!!!", group = "010")
public class RoadRunnerAuto extends LinearOpMode {

    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    ElapsedTime runtime = new ElapsedTime();
    double liftFeedForward = -.0006;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot.init(hardwareMap);
        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        waitForStart();

        Trajectory goForward = robot.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(30).build();
        robot.followTrajectory(goForward);
        robot.turn(Math.PI/2);
        //moveLift(300);
        Trajectory goToPole = robot.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(10).build();
        robot.followTrajectory(goToPole);
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

}
