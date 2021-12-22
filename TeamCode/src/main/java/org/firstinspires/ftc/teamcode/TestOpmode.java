package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp(name = "OpmodeTest")
public class TestOpmode extends LinearOpMode {
//0-136
   IrSeekerSensor irSeekerSensor;
   UltrasonicSensor ultrasonicSensor;
   Servo ser;
   double degree;
    @Override
    public void runOpMode() throws InterruptedException {
      //irSeekerSensor=hardwareMap.get(IrSeekerSensor.class,"irs");
      ser=hardwareMap.get(Servo.class,"Claw");
      //ultrasonicSensor=hardwareMap.get(UltrasonicSensor.class,"uls");
        waitForStart();
        while (opModeIsActive()){
           degree=ser.getPosition();
           telemetry.addData("Angle",degree);
            //telemetry.addData("Address",irSeekerSensor.getI2cAddress());
           //telemetry.addData("Target",Base.getTargetPosition());
           telemetry.update();
        }
    }
}
