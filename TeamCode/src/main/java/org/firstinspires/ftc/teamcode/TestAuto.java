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
        autohwp.Base.setTargetPosition(-4000);
        autohwp.Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autohwp.Base.setPower(0.5);
        while (autohwp.Base.isBusy()&&!autohwp.Left_touch.isPressed()){
            telemetry.addData("Target :",autohwp.Base.getTargetPosition());
            telemetry.addData("Current :",autohwp.Base.getCurrentPosition());
            telemetry.update();
        }
        autohwp.Base.setPower(0);
        autohwp.Base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Target :",autohwp.Base.getTargetPosition());
        telemetry.addData("Current :",autohwp.Base.getCurrentPosition());
        telemetry.update();
    }
}
