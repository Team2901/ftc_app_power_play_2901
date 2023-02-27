package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumTeleOp extends OpMode {

    /*
    * |L1   R1|
    * |L2   R2|
    * |L3   R3|
    * |L4   R4|
    * */
    DcMotor left1;
    DcMotor left2;
    DcMotor left3;
    DcMotor left4;
    DcMotor right1;
    DcMotor right2;
    DcMotor right3;
    DcMotor right4;

    BNO055IMU imu;

    @Override
    public void init() {
        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        left3 = hardwareMap.get(DcMotor.class, "left3");
        left4 = hardwareMap.get(DcMotor.class, "left4");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        right3 = hardwareMap.get(DcMotor.class, "right3");
        right4 = hardwareMap.get(DcMotor.class, "right4");
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        right3.setDirection(DcMotorSimple.Direction.REVERSE);
        right4.setDirection(DcMotorSimple.Direction.REVERSE);
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        double turnPower = gamepad1.right_stick_x;
        double forwardMovePower = -gamepad1.left_stick_y;
        double sideMovePower = gamepad1.left_stick_x;
        left1.setPower(forwardMovePower + turnPower + sideMovePower);
        left2.setPower(forwardMovePower + turnPower + sideMovePower);
        left3.setPower(forwardMovePower + turnPower - sideMovePower);
        left4.setPower(forwardMovePower + turnPower - sideMovePower);
        right1.setPower(forwardMovePower - turnPower - sideMovePower);
        right2.setPower(forwardMovePower - turnPower - sideMovePower);
        right3.setPower(forwardMovePower - turnPower + sideMovePower);
        right4.setPower(forwardMovePower - turnPower + sideMovePower);

    }
}
