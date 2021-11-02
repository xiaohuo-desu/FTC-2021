package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.util.Arrays;
@TeleOp(name="Car2")
public class Car2 extends LinearOpMode {

    DcMotorEx Leftfront;
    DcMotorEx Rightfront;
    DcMotorEx Leftback;
    DcMotorEx Rightback;

    double flm,frm,blm,brm;

    double Maxspeed=800;

    DcMotor rolling;
    DcMotorEx arm;

    CRServo catching;

    TouchSensor touchSensor;

    public enum Catch{
        Open,Close
    }

    public static PIDCoefficients pidCoeffs=new PIDCoefficients(0.6,0,0);
    public  PIDCoefficients pidGains= new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer= new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    @Override
    public void runOpMode() throws InterruptedException {

        Catch cath=Catch.Open;

        Rightback =hardwareMap.get(DcMotorEx.class,"RightBack");
        Leftback = hardwareMap.get(DcMotorEx.class,"LeftBack");
        Rightfront = hardwareMap.get(DcMotorEx.class,"RightFront");
        Leftfront = hardwareMap.get(DcMotorEx.class,"LeftFront");

        rolling=hardwareMap.get(DcMotor.class,"rolling");
        arm=hardwareMap.get(DcMotorEx.class,"arm");

        catching=hardwareMap.get(CRServo.class,"catching");

        //touchSensor=hardwareMap.get(TouchSensor.class,"TouchSensor");

        Leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightback.setDirection(DcMotorSimple.Direction.REVERSE);

        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //if(!touchSensor.isPressed()){
           // arm.setVelocity(-800);
        //}
        //else{
           // arm.setMotorDisable();
        //}

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Status", "Running");
            telemetry.update();

            arm.setMotorEnable();

            flm= (-gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*Maxspeed;
            frm= (-gamepad1.left_stick_y-gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*Maxspeed;
            blm= (-gamepad1.left_stick_y-gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*Maxspeed;
            brm= (-gamepad1.left_stick_y+gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*Maxspeed;

            PID(flm,Leftfront);
            PID(frm,Rightfront);
            PID(blm,Leftback);
            PID(brm,Rightback);

            //机械臂
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(gamepad1.left_trigger!=0){
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm.setDirection(DcMotorSimple.Direction.REVERSE);
                arm.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger!=0)
            {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm.setDirection(DcMotorSimple.Direction.FORWARD);
                arm.setPower(gamepad1.right_trigger);
            }
            //转盘
            if(gamepad1.right_bumper){
                rolling.setPower(0.5);
            }
            else {
                rolling.setPower(0);
            }
            //爪子

            double closeTime=50;

            switch (cath)
            {
                case Open:
                    if(gamepad1.b){
                        catching.setPower(0.3);
                        while(closeTime!=0){
                            closeTime--;
                        }
                        cath=Catch.Close;
                        closeTime=50;
                    }
                    break;
                case Close:
                    if(gamepad1.b) {
                        catching.setPower(-0.3);
                        while (closeTime!= 0) {
                            closeTime--;
                        }
                        cath=Catch.Open;
                        closeTime=50;
                    }
                    break;
            }
        }
    }

    double intergral= 0;
    double LastError=0;
    public void PID(double targetVelocity,DcMotorEx targetMotor)
    {
        PIDTimer.reset();

        double currentVelocity=targetMotor.getVelocity();
        double error= targetVelocity-currentVelocity;

        intergral+=error*PIDTimer.time();
        double deltaError=error-LastError;
        double derivative = deltaError/PIDTimer.time();  //the rate of change

        pidGains.p= pidCoeffs.p*error;
        pidGains.i=pidCoeffs.i*intergral;
        pidGains.d=pidCoeffs.d*derivative;

        targetMotor.setVelocity(pidGains.p+ pidGains.i+ pidGains.d+targetVelocity);

        LastError=error;

    }
}