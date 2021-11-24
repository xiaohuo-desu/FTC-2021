package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {
    Hardwaremap autohwp = new Hardwaremap();
    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);
        waitForStart();
        autohwp.Leftfront.setTargetPosition(2000);
        autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autohwp.Leftfront.setPower(0.5);
        while (autohwp.Leftfront.isBusy()){
            telemetry.addData("Target :",autohwp.Leftfront.getTargetPosition());
            telemetry.addData("Current :",autohwp.Leftfront.getCurrentPosition());
            telemetry.update();
        }
        autohwp.Leftfront.setPower(0);
        autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Target :",autohwp.Leftfront.getTargetPosition());
        telemetry.addData("Current :",autohwp.Leftfront.getCurrentPosition());
        telemetry.update();
    }
}
