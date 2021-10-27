package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClawTeleOp extends OpMode {
    DcMotor Leftfront;
    DcMotor Rightfront;
    DcMotor Leftback;
    DcMotor Rightback;

    Servo Claw;

    double LeFandRiBPower;
    double RiFandLeBPower;

    @Override
    public void init() {
        Leftback=hardwareMap.dcMotor.get("Left-Back");
        Rightback=hardwareMap.dcMotor.get("Right-Back");
        Leftfront=hardwareMap.dcMotor.get("Left-Front");
        Rightback=hardwareMap.dcMotor.get("Right-Front");

        Claw = hardwareMap.servo.get(("Holder"));

        Leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightback.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //start+A键为gamepad1    start+B键为gamepad2
        //手柄中上拨为-1，下拨为1
        LeFandRiBPower=-gamepad1.left_stick_y;
        RiFandLeBPower=-gamepad1.right_stick_y;
        Leftback.setPower(RiFandLeBPower);
        Rightback.setPower(LeFandRiBPower);
        Leftfront.setPower(LeFandRiBPower);
        Rightfront.setPower(RiFandLeBPower);

        if(gamepad1.x)
        {
            Claw.setPosition(0);
        }
        else
        {
            Claw.setPosition(1);
        }
    }
}
