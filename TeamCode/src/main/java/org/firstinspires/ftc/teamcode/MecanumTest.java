package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.Arrays;
@TeleOp(name="MecanumTeleOp")
public class MecanumTest extends LinearOpMode {

    DcMotorEx Leftfront;
    DcMotorEx Rightfront;
    DcMotorEx Leftback;
    DcMotorEx Rightback;

    double Left_stick_x;
    double Left_stick_y;
    double power;

    double angle;

    double flm,frm,blm,brm;

    double Maxspeed=1;

    DcMotor rolling;
    DcMotor arm;

    Servo catching;

    public enum Catch{
        Open,Close
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Rightback =hardwareMap.get(DcMotorEx.class,"RightBack");
        Leftback = hardwareMap.get(DcMotorEx.class,"LeftBack");
        Rightfront = hardwareMap.get(DcMotorEx.class,"RightFront");
        Leftfront = hardwareMap.get(DcMotorEx.class,"LeftFront");

        rolling=hardwareMap.get(DcMotor.class,"rolling");
        arm=hardwareMap.get(DcMotor.class,"arm");

        catching=hardwareMap.get(Servo.class,"catching");

        Leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightback.setDirection(DcMotorSimple.Direction.FORWARD);

        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        catching.scaleRange(0,0.6);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Status", "Running");
            telemetry.update();

            Left_stick_x=gamepad1.left_stick_x;
            Left_stick_y=gamepad1.left_stick_y;

            angle=Math.atan2(Left_stick_x,Left_stick_y);

            power=Math.sqrt(Math.pow(Left_stick_x+Left_stick_y,2));

            double radiantodirection=Math.toRadians(angle);

            flm=(Math.cos(radiantodirection-Math.sin(radiantodirection)))*power;
            frm=(Math.cos(radiantodirection+Math.sin(radiantodirection)))*power;
            blm=(Math.cos(radiantodirection+Math.sin(radiantodirection)))*power;
            brm=(Math.cos(radiantodirection-Math.sin(radiantodirection)))*power;

            double powerreducer=1;

            double [] numbers={flm,frm,blm,brm};

            Arrays.sort(numbers);

            double min=numbers[0];
            double max=numbers[numbers.length-1];

            if((Math.abs(max)>Maxspeed)||(Math.abs(min)>Maxspeed)){
                if(Math.abs(max)>=Math.abs(min)){
                    powerreducer=(Maxspeed/max);
                }else{
                    powerreducer=(Maxspeed/Math.abs(min));
                }
            }

            flm=flm*powerreducer;
            frm=flm*powerreducer;
            blm=flm*powerreducer;
            brm=flm*powerreducer;

            flm=Math.round(flm*1000.0)/1000.0;
            frm=Math.round(flm*1000.0)/1000.0;
            blm=Math.round(flm*1000.0)/1000.0;
            brm=Math.round(flm*1000.0)/1000.0;

            Leftfront.setPower(flm);
            Rightfront.setPower(frm);
            Leftback.setPower(blm);
            Rightback.setPower(brm);

            //机械臂
            arm.setPower(gamepad1.left_trigger);
            arm.setPower(-gamepad1.right_trigger);
            //转盘
            if(gamepad1.right_bumper){
                rolling.setPower(0.5);
            }
            else {
                rolling.setPower(0);
            }
            //爪子

            switch ()

        }
    }
}
