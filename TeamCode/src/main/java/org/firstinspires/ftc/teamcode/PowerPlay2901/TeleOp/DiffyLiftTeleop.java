package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

@TeleOp(name = "Diffy Lift Teleop", group = "AAAAAAAAAAAAhRockBot")
public class DiffyLiftTeleop extends OpMode {
    RockBotHardware robot = new RockBotHardware();

    double liftPower = 0;
    double extendPower = 0;
    double feedForward = .3;
    int liftTarget = 85;
    int extendTarget = 0;
    double leftPodPower = 0;
    double rightPodPower = 0;
    public double leftTurnPower = 0;
    public double rightTurnPower = 0;
    double moveAngle;
    double speedMod = 1.8;
    double resetSpeedMod = 1.8;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    @Override
    public void loop() {
        double forwardPower = -gamepad1.left_stick_y;
        double sidePower = -gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x - .2*sidePower;
        if(gamepad2.left_trigger > .5) {
            forwardPower = 0;
            sidePower = 0;
            turnPower = 0;
        }


        moveAngle = Math.toDegrees(Math.atan2(sidePower, -forwardPower+.001));
        //moveAngle = AngleUnit.normalizeDegrees(moveAngle+robot.getAngle()); //uncomment this for field oriented
        leftPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)+(.7*turnPower*Math.cos(Math.toRadians(moveAngle)));
        rightPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)-(.7*turnPower*Math.cos(Math.toRadians(moveAngle)));
        leftTurnPower = leftPodTurn(moveAngle-(45*turnPower*Math.sin(Math.toRadians(moveAngle))));
        rightTurnPower = rightPodTurn(moveAngle+(45*turnPower*Math.sin(Math.toRadians(moveAngle))));

        if(gamepad2.left_trigger > 0.5){
            robot.claw.setPosition(.915);
            robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        } else {
            robot.claw.setPosition(.85);
            robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }

        if(gamepad2.y){
            liftTarget = 815;
            liftI = 0;
        }
        if(gamepad2.x) {
            liftTarget = 575;
            liftI = 0;
        }
        if(gamepad2.b) {
            liftTarget = 325;
            liftI = 0;
        }
        if(gamepad2.a){
            liftTarget = 85;
            liftI = 0;
        }
        if(gamepad2.dpad_left){
            liftTarget = 165;
            liftI = 0;
        }

        if(gamepad1.left_bumper){
            speedMod = 1;
        } else {
            speedMod = resetSpeedMod;
        }

        if(gamepad1.b){
            resetSpeedMod = 3;
        } else if(gamepad1.x){
            resetSpeedMod = 1.8;
        }

        extendPower = extendPower(extendTarget);

        if(gamepad2.right_trigger > .5){
            liftPower = liftPower(liftTarget - 85);
            feedForward = 0;
            liftI = 0;
        } else {
            liftPower = liftPower(liftTarget);
            feedForward = .3;
            telemetry.addData("liftPower", liftPower);
        }

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;

        robot.liftOne.setPower((liftPower + extendPower - feedForward) * scaleFactor);
        robot.liftTwo.setPower((liftPower - extendPower - feedForward) * scaleFactor);

        robot.leftOne.setVelocity((leftPodPower/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((leftPodPower/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((rightPodPower/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((rightPodPower/speedMod-rightTurnPower)*2500);

        telemetry.addData("Left Pod Angle", leftPodAngle);
        telemetry.addData("Right Pod Angle", rightPodAngle);
        telemetry.addData("Lift Position", robot.liftOne.getCurrentPosition());
        telemetry.addData("joy angle", moveAngle);
        telemetry.update();
    }

    double kp = 1.2;
    double ki = 0;
    double kd = 0;

    private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    public double leftPodTurn(double angle){
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            leftPodPower = -leftPodPower;
        }
        double secs = runtimePodLeft.seconds();
        runtimePodLeft.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);
        pAngleLeft = error;
        double total = (kp* pAngleLeft + ki* iAngleLeft + kd* dAngleLeft)/100;
        if(total > 1){
            iAngleLeft = 0;
            total = 1;
        }
        if(total < -1){
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle){
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition())/8.925;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            rightPodPower = -rightPodPower;
        }
        double secs = runtimePodRight.seconds();
        runtimePodRight.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);
        pAngleRight = error;
        double total = (kp* pAngleRight + ki* iAngleRight + kd* dAngleRight)/100;
        if(total > 1){
            iAngleRight = 0;
            total = 1;
        }
        if(total < -1){
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }

    double klp = 0.7;
    double kli = 0.0005;
    double kld = 0.015;

    public ElapsedTime runtimeLift = new ElapsedTime();
    double liftPosition = 0;
    double liftP = 0;
    double liftI = 0;
    double liftD = 0;

    public double liftPower(int target){
        int error = (robot.liftTwo.getCurrentPosition() + robot.liftOne.getCurrentPosition())/2 - target;
        telemetry.addData("lift error", error);
        double secs = runtimeLift.seconds();
        telemetry.addData("lift loop time", secs);
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

    double kep = 0;
    double kei = 0;
    double ked = 0;
    ElapsedTime runtimeExtend = new ElapsedTime();
    double extendP = 0;
    double extendI = 0;
    double extendD = 0;

    public double extendPower(int target){
        int error = (robot.liftTwo.getCurrentPosition() - robot.liftOne.getCurrentPosition())/2 - target;
        telemetry.addData("extend error", error);
        double secs = runtimeExtend.seconds();
        telemetry.addData("extend loop time", secs);
        runtimeExtend.reset();
        extendD = (error - extendP) / secs;
        extendI = extendI + (error * secs);
        extendP = error;
        double total = (kep* extendP + kei* extendI + ked* extendD)/100;
        if(total > 1){
            extendI = 0;
            total = 1;
        }
        if(total < -1){
            extendI = 0;
            total = -1;
        }
        return total;
    }
}
