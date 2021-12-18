package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Hardwaremap {
    DcMotorEx Leftfront=null; //1
    DcMotorEx Rightfront=null; //0
    DcMotorEx Leftback=null; //2
    DcMotorEx Rightback=null; //3
    DcMotorEx Elevator=null;
    DcMotorEx Base=null;
    DcMotor Rolling=null;
    Servo Claw=null;
    TouchSensor Left_touch=null;
    TouchSensor Right_touch=null;

    HardwareMap hwp=null;

    public Hardwaremap(){}

    public void init(HardwareMap HwMap){
        hwp=HwMap;

        Rightback = hwp.get(DcMotorEx.class, "RightBack");
        Leftback = hwp.get(DcMotorEx.class, "LeftBack");
        Rightfront = hwp.get(DcMotorEx.class, "RightFront");
        Leftfront = hwp.get(DcMotorEx.class, "LeftFront");
        Elevator=hwp.get(DcMotorEx.class,"Elevator");
        Base=hwp.get(DcMotorEx.class,"Base");
        Rolling=hwp.dcMotor.get("Rolling");
        Claw = hwp.servo.get(("Claw"));
        Left_touch=hwp.get(TouchSensor.class,"Lefttouch");
        Right_touch=hwp.get(TouchSensor.class,"Righttouch");

        Leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        Base.setDirection(DcMotorSimple.Direction.REVERSE);
        Elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        Claw.setDirection(Servo.Direction.FORWARD);
        
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rolling.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
