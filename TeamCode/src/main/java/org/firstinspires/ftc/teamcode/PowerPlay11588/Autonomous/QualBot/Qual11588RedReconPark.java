package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Disabled
@Autonomous(name = "Qual 11588 Red Recon-Park ", group = "11588")
public class Qual11588RedReconPark extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(this.hardwareMap, telemetry, Qual11588Hardware.allianceColor.RED);
        waitForStart();
        robot.pipeLine.startCam();
        timer.reset();
        while(robot.pipeLine.framesProceeded < 30 && timer.milliseconds() < 5000){

        }
        moveArm(Height.MEDIUM);
        reconParkAuto();
        timer.reset();
        while (timer.milliseconds() > 1000) {}
    }
}
