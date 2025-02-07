package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name = "Qual 11588", group = "11588")
public class Qual11588TeleOp extends OpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public enum ClawPosition{Open, Closed}
    ClawPosition currentClawPosition = ClawPosition.Closed;
    public enum Height{GROUND, LOW, MEDIUM, HIGH}
    Height currentArmHeight = Height.GROUND;
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;

    //All the variables that are needed for pid
    ElapsedTime PIDTimer = new ElapsedTime();
    int armTarget = 200;

    //Making different variables for each target height
    Height lastArmHeight = currentArmHeight;
    int groundPolePosition = 200;
    int lowPolePosition = 550;
    int mediumPolePosition = 800;
    int highPolePosition = 1150;
    int zeroAngleTicks = lowPolePosition - 150;

    double error = 0.0;
    double total = 0.0;
    double kp = 0.9;
    double ki = 0.0;
    double kd = 0.0;
    double kCos = 0.3;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double cosArm = 0.0;
    double iArmMax = .25;
    double armAngle = 0;

    @Override
    public void init() {
        robot.teleOpInit(hardwareMap, telemetry, false);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();
        if(impGamepad1.dpad_left.isInitialPress()){
            //Sets the armTarget to ground/intake
            armTarget = groundPolePosition;
            currentArmHeight = Height.GROUND;
        }else if(impGamepad1.dpad_down.isInitialPress()){
            //Sets the armTarget to the low pole
            armTarget = lowPolePosition;
            currentArmHeight = Height.LOW;
        }else if(impGamepad1.dpad_right.isInitialPress()){
            //Sets the armTarget to the mid pole
            armTarget = mediumPolePosition;
            currentArmHeight = Height.MEDIUM;
        }else if(impGamepad1.dpad_up.isInitialPress()){
            //Sets the armTarget to the high pole
            armTarget = highPolePosition;
            currentArmHeight = Height.HIGH;
        }
        /*Allows for the armTarget to be changed for the duration of the TeleOp rather than resetting
        when you change height*/
        if(impGamepad1.y.isInitialPress()){
            if(currentArmHeight == Height.GROUND){
                groundPolePosition += 10;
            }else if(currentArmHeight == Height.LOW){
                lowPolePosition += 10;
            }else if(currentArmHeight == Height.MEDIUM){
                mediumPolePosition += 10;
            }else if(currentArmHeight == Height.HIGH){
                highPolePosition += 10;
            }
        }
        if(impGamepad1.a.isInitialPress()){
            if(currentArmHeight == Height.GROUND){
                groundPolePosition -= 10;
            }else if(currentArmHeight == Height.LOW){
                lowPolePosition -= 10;
            }else if(currentArmHeight == Height.MEDIUM){
                mediumPolePosition -= 10;
            }else if(currentArmHeight == Height.HIGH){
                highPolePosition -= 10;
            }
        }

        if(currentArmHeight == Height.GROUND){
            armTarget = groundPolePosition;
        }else if(currentArmHeight == Height.LOW){
            armTarget = lowPolePosition;
        }else if(currentArmHeight == Height.MEDIUM){
            armTarget = mediumPolePosition;
        }else if(currentArmHeight == Height.HIGH){
            armTarget = highPolePosition;
        }

        if(impGamepad1.x.isInitialPress()){
            lowPolePosition = robot.arm.getCurrentPosition();
            zeroAngleTicks = lowPolePosition - 75;
            groundPolePosition = lowPolePosition - 350;
            mediumPolePosition = lowPolePosition +  250;
            highPolePosition = lowPolePosition + 600;
        }

        robot.arm.setPower(armPower(armTarget));

        if(impGamepad1.back.isInitialPress()){
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(impGamepad1.right_trigger.getValue() > 0){
            turningPower = .3 * impGamepad1.right_trigger.getValue();
        }else if(impGamepad1.left_trigger.getValue() > 0){
            turningPower = -.3 * impGamepad1.left_trigger.getValue();
        }else{
            turningPower = .75 * impGamepad1.right_stick_x.getValue();
        }
        double y = .75 * impGamepad1.left_stick_y.getValue();
        double x = .75 * impGamepad1.left_stick_x.getValue();
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        switch (currentClawPosition){
            case Open:
                robot.claw.setPosition(Qual11588Hardware.OPEN_POSITION);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = ClawPosition.Closed;
                }
                break;
            case Closed:
                robot.claw.setPosition(robot.CLOSED_POSITION);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = ClawPosition.Open;
                }
        }
        telemetryStuff();
    }

    public double armPower(int target){

        error = target - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / PIDTimer.seconds();
        iArm = iArm + (error * PIDTimer.seconds());
        pArm = error;
        armAngle = recalculateAngle();
        cosArm = Math.cos(Math.toRadians(armAngle));
        total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100 + (cosArm * kCos);
        PIDTimer.reset();

        if(currentArmHeight != lastArmHeight){
            iArm = 0;
        }

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }

        if(total > .5){
            total = .5;
        }
        if(recalculateAngle() > 60 && total < -.5){
            total = -.5;
        }else if(total < .005 && recalculateAngle() < 60){
            total = .005;
        }
        lastArmHeight = currentArmHeight;

        return total;
    }

    public double recalculateAngle(){
        //Placeholder variables that will be deleted
        double rightAngleDiff = 800;
        double slope = 90/((zeroAngleTicks + rightAngleDiff) - zeroAngleTicks);
        double newAngle = slope * (robot.arm.getCurrentPosition() - zeroAngleTicks);
        return newAngle;
    }

    public void telemetryStuff(){
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        telemetry.addData("Claw State", currentClawPosition);
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Angle", recalculateAngle());
        telemetry.addData("Current Target Height", currentArmHeight);
        telemetry.addData("Ground Position", groundPolePosition);
        telemetry.addData("Low Position", lowPolePosition);
        telemetry.addData("Medium Position", mediumPolePosition);
        telemetry.addData("High Position", highPolePosition);
        telemetry.addData("PID Total", total);
        telemetry.addData("P Arm", pArm);
        telemetry.addData("I Arm", iArm);
        telemetry.addData("D Arm", dArm);
        telemetry.addData("Cos Arm", cosArm);
        telemetry.addData("Proportional Stuff", pArm * kp);
        telemetry.addData("Integral Stuff", iArm * ki);
        telemetry.addData("Derivative Stuff", dArm * kd);
        telemetry.addData("Cos Stuff", cosArm * kCos);
        telemetry.update();
    }
}
