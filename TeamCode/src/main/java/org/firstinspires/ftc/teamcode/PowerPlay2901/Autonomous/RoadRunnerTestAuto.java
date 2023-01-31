package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;

@Autonomous(name = "roadrunner test", group = "010")
public class RoadRunnerTestAuto extends LinearOpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot.init(hardwareMap);
        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d();
        TrajectorySequence moveForwards = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(20, 0), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(0, -20), Math.toRadians(90))
                .build();
        waitForStart();

        robot.followTrajectorySequence(moveForwards);
    }
}
