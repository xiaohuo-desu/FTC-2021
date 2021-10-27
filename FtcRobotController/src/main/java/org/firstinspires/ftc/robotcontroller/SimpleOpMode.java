package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Skeleton Op Mode", group="/")
public class SimpleOpMode extends OpMode {
    DcMotor Leftfront;
    DcMotor Rightfront;
    DcMotor Leftback;
    DcMotor Rightback;
    double Power=0.4;

    @Override
    public void init()
    {
        Leftback=hardwareMap.dcMotor.get("Left-Back");
        Rightback=hardwareMap.dcMotor.get("Right-Back");
        Leftfront=hardwareMap.dcMotor.get("Left-Front");
        Rightback=hardwareMap.dcMotor.get("Right-Front");
        Leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightback.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop()
    {
        Leftback.setPower(Power);
        Rightback.setPower(Power);
        Leftfront.setPower(Power);
        Rightfront.setPower(Power);
    }

    @Override
    public void stop(){

    }
}