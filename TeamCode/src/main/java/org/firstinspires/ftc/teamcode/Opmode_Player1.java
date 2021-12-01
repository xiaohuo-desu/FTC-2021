package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="19851TeleOp_Player1")
public class Opmode_Player1 extends LinearOpMode {

    double M1,M2,M3,M4;
    double MotorMaxspeed=0.8;
    double position=0.6;
    double startPosition;

    double Deltime;

    boolean Isload=true;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    public static PIDCoefficients pidCoeffs=new PIDCoefficients(0.6,0,0);
    public  PIDCoefficients pidGains= new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer= new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Hardwaremap telehwp = new Hardwaremap();

    Velocity v;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        resetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            getAngle();

            imu.startAccelerationIntegration(imu.getPosition(), v,1);

            telemetry.addData("Status", "Running");
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("Velocity",imu.getVelocity());
            telemetry.addData("position",imu.getPosition());
            telemetry.update();

            //start+A键为gamepad1    start+B键为gamepad2
            //手柄中上拨为-1，下拨为1
            M1= (-gamepad1.left_stick_y-gamepad1.left_stick_x-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M2= (-gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x/2))*MotorMaxspeed;
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
            /*if(gamepad1.b)
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

            telemetry.addData("start position",startPosition);*/
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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



}