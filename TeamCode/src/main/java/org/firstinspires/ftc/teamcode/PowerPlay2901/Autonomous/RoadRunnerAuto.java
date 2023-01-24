package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.drive.SampleTankDrive;

@Autonomous(name = "ROAD RUNNER AUTO!!!", group = "010")
public class RoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        waitForStart();

        Trajectory goForward = robot.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(48).build();
        robot.followTrajectory(goForward);


    }
}
