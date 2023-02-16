package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

@TeleOp(name = "servo position finder", group = "test")
public class ServoPositionFinderTeleOp extends OpMode {
    RockBotHardware robot = new RockBotHardware();
    double position;

    @Override
    public void init() {
        robot.init(this.hardwareMap);
        position = robot.claw.getPosition();
        robot.underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    @Override
    public void loop() {
        position += gamepad1.left_stick_y/10000;
        telemetry.addData("position", position);
        telemetry.update();
        robot.claw.setPosition(position);
    }
}
