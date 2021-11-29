package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="19851TeleOp")
public class Opmode extends LinearOpMode {

    double M1,M2,M3,M4;
    double MotorMaxspeed=0.8;
    double position=0.6;
    double startPosition;

    double Deltime;

    boolean Isload=true;

    public static PIDCoefficients pidCoeffs=new PIDCoefficients(0.6,0,0);
    public  PIDCoefficients pidGains= new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer= new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Hardwaremap telehwp = new Hardwaremap();

    @Override
    public void runOpMode() throws InterruptedException {

        telehwp.init(hardwareMap);

        telehwp.Claw.setPosition(0);
        startPosition=telehwp.Claw.getPosition();
        Deltime=0;

        telehwp.Leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telehwp.Leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telehwp.Rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telehwp.Rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();
            //start+A键为gamepad1    start+B键为gamepad2
            //手柄中上拨为-1，下拨为1
            M1= (gamepad1.left_stick_y-gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M2= (gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M3= (-gamepad1.left_stick_y+gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M4= (-gamepad1.left_stick_y-gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*MotorMaxspeed;

            telehwp.Leftfront.setPower(M1);
            telehwp.Rightfront.setPower(M2);
            telehwp.Leftback.setPower(M3);
            telehwp.Rightback.setPower(M4);

            /*PID(M1,telehwp.Leftfront);
            PID(M2,telehwp.Rightfront);
            PID(M3,telehwp.Leftback);
            PID(M4,telehwp.Rightback);*/

            //翻斗
            if(gamepad1.b)
            {
                telehwp.Claw.setPosition(position);
                sleep(1500);
                telehwp.Claw.setPosition(startPosition);
                Deltime=0;
            }

            //升降
            telehwp.Elevator.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            if(gamepad1.left_trigger==0||gamepad1.right_trigger==0){
                telehwp.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            double ElePowerUp=gamepad1.left_trigger;
            if(ElePowerUp!=0&&telehwp.Claw.getPosition()==startPosition&&Isload==true){
                Deltime+=PIDTimer.time();
                if(Deltime>100)
                {
                    telehwp.Claw.setPosition(0.2);
                    Isload=false;
                }
            }
            double ElePowerDown=-gamepad1.right_trigger;
            if(ElePowerDown!=0)
            {
                Isload=true;
            }

            //转盘

            if(gamepad1.left_bumper)
            {
                telehwp.Rolling.setPower(-0.5);
            }
            else{
                telehwp.Rolling.setPower(0);
            }

            //吸取

            if(gamepad1.right_bumper)
            {
                telehwp.Cubecatcher.setPower(-1);
            }
            else
            {
                telehwp.Cubecatcher.setVelocity(0);
            }

            //数值显示测试

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