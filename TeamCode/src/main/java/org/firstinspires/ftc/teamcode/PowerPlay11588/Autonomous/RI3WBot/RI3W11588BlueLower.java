package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.RI3WBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.RI3WBot.RI3W11588BaseAutonomous;

@Disabled
@Autonomous(name = "RI3W 11588 Blue Lower v3", group = "11588")
public class RI3W11588BlueLower extends RI3W11588BaseAutonomous {
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        waitForStart();
        runTime.reset();
        moveArm(Height.GROUND);
        while(runTime.milliseconds() < 2000){

        }
        park();
    }
}
