package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "19581TeleOp_Player2")
public class Opmode_Player2 extends LinearOpMode {

    Hardwaremap telehwp = new Hardwaremap();

    int Initial_Height;
    int Safe_Height;
    int Current_Height;

    boolean IsTurnningOk;

    @Override
    public void runOpMode() throws InterruptedException {
        telehwp.init(hardwareMap);
        telehwp.Claw.setPosition(0);
        // TODO: 2021/12/1 初始化机械臂
         Initial_Height=telehwp.Elevator.getCurrentPosition();
        // TODO: 2021/12/1 设置高度阈值
        IsTurnningOk=false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            //情况判断
            Current_Height=telehwp.Elevator.getCurrentPosition();
            if(Current_Height>Safe_Height)
                IsTurnningOk=true;
            // TODO: 2021/12/1 转向判断
            //转向
             telehwp.Base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             if(gamepad2.left_stick_x<0&&IsTurnningOk)
                 telehwp.Base.setPower(-0.1);
             else if(gamepad2.left_stick_x>0&&IsTurnningOk)
                 telehwp.Base.setPower(0.1);
             else
                 telehwp.Base.setPower(0);
             //抬升
            telehwp.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad2.right_stick_y<0)
                telehwp.Elevator.setPower(-0.1);
            else if(gamepad2.right_stick_y>0)
                telehwp.Elevator.setPower(0.1);
            else
                telehwp.Elevator.setPower(0);
            //夹子
            //转盘
        }
    }
}
