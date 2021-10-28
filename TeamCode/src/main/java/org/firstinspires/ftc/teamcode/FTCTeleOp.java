package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="19851TeleOp")
public class FTCTeleOp extends LinearOpMode {
    DcMotorEx Leftfront;
    DcMotorEx Rightfront;
    DcMotorEx Leftback;
    DcMotorEx Rightback;
    DcMotorEx Elevator;
    DcMotor Rolling;
    DcMotorEx Cubecatcher;

    Servo Claw;

    double M1,M2,M3,M4;
    double MotorMaxspeed=800;
    double position=0.6;
    double startPosition;

    double ElevatorPower;
    double ElevatorPosition;
    double Deltime;

    boolean Isload=true;


    public static PIDCoefficients pidCoeffs=new PIDCoefficients(0,0,0);
    public  PIDCoefficients pidGains= new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer= new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() throws InterruptedException {

        Rightback =hardwareMap.get(DcMotorEx.class,"RightBack");
        Leftback = hardwareMap.get(DcMotorEx.class,"LeftBack");
        Rightfront = hardwareMap.get(DcMotorEx.class,"RightFront");
        Leftfront = hardwareMap.get(DcMotorEx.class,"LeftFront");

        Elevator=hardwareMap.get(DcMotorEx.class,"Elevator");
        Rolling=hardwareMap.dcMotor.get("Rolling");
        Cubecatcher=hardwareMap.get(DcMotorEx.class,"CatchCube");
        Claw = hardwareMap.servo.get(("Holder"));

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

        Cubecatcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Cubecatcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        startPosition=Claw.getPosition();
        Deltime=0;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();
            //start+A键为gamepad1    start+B键为gamepad2
            //手柄中上拨为-1，下拨为1
            M1= (-gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M2= (-gamepad1.left_stick_y-gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M3= (-gamepad1.left_stick_y-gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M4= (-gamepad1.left_stick_y+gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*MotorMaxspeed;

            PID(M1,Leftfront);
            PID(M2,Rightfront);
            PID(M3,Leftback);
            PID(M4,Rightback);

            //翻斗
            if(gamepad1.b)
            {
                Claw.setPosition(position);
                sleep(1500);
                Claw.setPosition(startPosition);
                Deltime=0;
            }

            //升降

            double ElePowerUp=gamepad1.left_trigger;
            if(ElePowerUp!=0&&Claw.getPosition()==startPosition&&Isload==true){
                Deltime+=PIDTimer.time();
                if(Deltime>100)
                {
                    Claw.setPosition(0.2);
                    Isload=false;
                }
            }
            double ElePowerDown=-gamepad1.right_trigger;
            if(ElePowerDown!=0){
                Isload=true;
            }



            //转盘

            if(gamepad1.left_bumper)
            {
                Rolling.setPower(-0.5);
            }
            else{
                Rolling.setPower(0);
            }

            //吸取

            if(gamepad1.right_bumper)
            {
                Cubecatcher.setVelocity(-100);
            }
            else
            {
                Cubecatcher.setVelocity(0);
            }
            //读取抬升数值
            ElevatorPower = Elevator.getPower();
            ElevatorPosition += Elevator.getPower();
            int limit = 5; //限位值

            //限位逻辑

            if(ElevatorPosition <= limit) //如果达到限制
            {
                Elevator.setPower((ElePowerUp+ElePowerDown)*0.5); //正常抬升
            }
            else
            {
                Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //强制制动，停止移动
                Elevator.setPower((ElePowerDown)*0.5); //仅允许向下
            }

            //数值显示测试
            telemetry.addData("抬升器数值（获取Power）：",ElevatorPower);
            telemetry.addData("抬升器相对位置：",ElevatorPosition);
            telemetry.addData("start position",startPosition);
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









