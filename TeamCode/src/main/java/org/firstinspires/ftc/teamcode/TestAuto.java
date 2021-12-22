package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {
    Hardwaremap autohwp = new Hardwaremap();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);
        waitForStart();
        /*autohwp.Elevator.setTargetPosition(-2000);
        autohwp.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autohwp.Elevator.setPower(0.2);
        while (autohwp.Elevator.isBusy()&&runtime.seconds()<2){
            telemetry.addData("Target :",autohwp.Elevator.getTargetPosition());
            telemetry.addData("Current :",autohwp.Elevator.getCurrentPosition());
            telemetry.update();
        }
        autohwp.Elevator.setPower(0);
        autohwp.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        /*autohwp.Claw.setPosition(0);
        sleep(500);
        autohwp.Claw.setPosition(0.1);
        sleep(500);
        autohwp.Claw.setPosition(0.2);
        sleep(500);
        autohwp.Claw.setPosition(0.3);
        sleep(500);
        autohwp.Claw.setPosition(0.4);
        sleep(500);*/
        autohwp.Claw.setPosition(0.5);
        sleep(500);
        autohwp.Claw.setPosition(0.8);
        sleep(500);
       /* autohwp.Claw.setPosition(0.6);
        sleep(500);
        autohwp.Claw.setPosition(0.7);
        sleep(500);
        autohwp.Claw.setPosition(0.8);
        sleep(500);
        autohwp.Claw.setPosition(0.9);
        sleep(500);
        autohwp.Claw.setPosition(1);
        sleep(500);*/
        //telemetry.addData("Target :",runtime.seconds());
        //telemetry.addData("Current :",autohwp.Base.getCurrentPosition());
        telemetry.update();
    }
}
