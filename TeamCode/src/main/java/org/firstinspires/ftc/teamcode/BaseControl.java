package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="BaseControl")
public class BaseControl extends LinearOpMode {
    TouchSensor Left;
    TouchSensor Right;
    DcMotorEx Base;
    @Override
    public void runOpMode() throws InterruptedException {
        Left=hardwareMap.get(TouchSensor.class,"Lefttouch");
        Right=hardwareMap.get(TouchSensor.class,"Righttouch");
        Base=hardwareMap.get(DcMotorEx.class,"Base");
        waitForStart();
        while (opModeIsActive()){
            if(Left.isPressed())
                Base.setPower(-0.5);
            else if(Right.isPressed())
                Base.setPower(0.5);
            else
                Base.setPower(0);
        }
    }
}
