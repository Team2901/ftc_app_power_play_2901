package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Autonomous(name = "ROAD RUNNER AUTO!!!", group = "010")
public class RoadRunnerAuto extends LinearOpMode {

    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    double outputLeft = 0;
    double outputRight = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot.init(hardwareMap);
        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        waitForStart();

        Trajectory goForward = robot.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(30).build();
        robot.followTrajectory(goForward);


    }


}
