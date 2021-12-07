package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "OpmodeTest")
public class TestOpmode extends LinearOpMode {
//0-136
   DcMotorEx Base;
    @Override
    public void runOpMode() throws InterruptedException {
       Base=hardwareMap.get(DcMotorEx.class,"Testmotor");
       Base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
           Base.setTargetPosition(20);
           Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           Base.setPower(gamepad1.left_stick_x);
           telemetry.addData("CurrentPosition",Base.getCurrentPosition());
           telemetry.addData("Target",Base.getTargetPosition());
           telemetry.update();
        }
    }
}
