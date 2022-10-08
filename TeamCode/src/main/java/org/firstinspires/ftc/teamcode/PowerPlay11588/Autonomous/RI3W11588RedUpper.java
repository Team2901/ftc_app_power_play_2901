package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV.*;

@Autonomous(name = "RI3W 11588 Red Upper", group = "11588")
public class RI3W11588RedUpper extends RI3W11588BaseAutonomous{


    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        runTime.reset();
        while(runTime.milliseconds() < 8000){}
        park();
    }
}
