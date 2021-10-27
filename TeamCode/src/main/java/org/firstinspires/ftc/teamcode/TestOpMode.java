package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="TestOpMode")
public class TestOpMode extends LinearOpMode {
    //    M1    M4
    //    ||    ||
    //    ||    ||
    //    M2====M3

    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M4;

    Servo Testservo;

    double M1power,M2power,M3power,M4power;

    @Override
    public void runOpMode() throws InterruptedException {


        double position=0.5;

        Testservo=hardwareMap.servo.get("Holder");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Status", "Running");
            telemetry.update();




            if(gamepad1.b){
                Testservo.setPosition(0.6);
                //position=position+0.5;
            }


        }
    }
}
