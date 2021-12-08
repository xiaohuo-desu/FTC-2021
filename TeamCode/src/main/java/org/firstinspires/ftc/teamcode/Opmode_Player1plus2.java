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

@TeleOp(name="19851TeleOp_Player1+2")
public class Opmode_Player1plus2 extends LinearOpMode {

    double M1,M2,M3,M4;
    double MotorMaxspeed=0.7;
    double position=0.6;
    double startPosition;

    double Deltime;

    boolean Isload=true;
    boolean isOpen;

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
        telehwp.Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telehwp.Base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        isOpen=false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            getAngle();

            imu.startAccelerationIntegration(imu.getPosition(), v,1);

            telemetry.addData("Status", "Running");
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("Lefttouch",telehwp.Left_touch.isPressed());
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

            //---------------Player2---------------------//

            //转向
            telehwp.Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //telehwp.Base.setPower(gamepad2.left_stick_x);
            if(!telehwp.Left_touch.isPressed()&&!telehwp.Right_touch.isPressed()){
                telehwp.Base.setPower(gamepad2.left_stick_x);
            }
            else if(telehwp.Left_touch.isPressed()){
                if(gamepad2.left_stick_x<0)
                    telehwp.Base.setPower(0);
                else
                    telehwp.Base.setPower(gamepad2.left_stick_x);
            }
            else {
                if(gamepad2.left_stick_x>0)
                    telehwp.Base.setPower(0);
                else
                    telehwp.Base.setPower(gamepad2.left_stick_x);
            }


            //抬升
            telehwp.Elevator.setPower(gamepad2.right_stick_y);


            //夹子
            if(isOpen&&gamepad2.a){
                telehwp.Claw.setPosition(0);
                isOpen=false;
            }
            if(!isOpen&&gamepad2.b){
                telehwp.Claw.setPosition(0.3);
                isOpen=true;
            }

            //转盘
            while (gamepad2.y){
                telehwp.Rolling.setPower(1);
            }
            telehwp.Rolling.setPower(0);
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