package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;

@Autonomous(name = "RuntoPositionTest")
public class TestRuntoPosition extends LinearOpMode {

    private DcMotor Base;

    @Override
    public void runOpMode() {
        /*telemetry.addData("Manufacturer",Base.getManufacturer());
        telemetry.addData("Type",Base.getMotorType());
        telemetry.addData("name",Base.getDeviceName());
        telemetry.update();*/
        Base = hardwareMap.get(DcMotor.class, "Testmotor");
        Base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Base.setDirection(DcMotorSimple.Direction.FORWARD);
        // Put initialization blocks here.
        waitForStart();
        Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Base.setTargetPosition(10);
        Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Base.setPower(1);
        while (Base.isBusy()){
            telemetry.addData("Current",Base.getCurrentPosition());
            telemetry.addData("Target",Base.getTargetPosition());
            telemetry.update();
        }
        Base.setPower(0);
        Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}