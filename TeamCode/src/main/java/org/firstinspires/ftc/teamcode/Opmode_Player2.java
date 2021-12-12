package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.guieffect.qual.SafeType;

//@TeleOp(name = "19581TeleOp_Player2")
public class Opmode_Player2 extends LinearOpMode {

    Hardwaremap telehwp = new Hardwaremap();

    int Initial_Height;
    int Safe_Height;
    int Current_Height;
    int Initial_Turn;
    int Current_Turn;

    double Initial_Claw;

    boolean isOpen;

    boolean IsTurnningOk;

    @Override
    public void runOpMode() throws InterruptedException {
        telehwp.init(hardwareMap);
        telehwp.Claw.setPosition(0);
        // TODO: 2021/12/1 初始化机械臂
        telehwp.Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telehwp.Base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Initial_Height=telehwp.Elevator.getCurrentPosition();
        Initial_Turn=telehwp.Base.getCurrentPosition();
        Initial_Claw=telehwp.Claw.getPosition();
        // TODO: 2021/12/1 设置高度阈值
        //Safe_Height=;
        IsTurnningOk=true;
        isOpen=false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            //情况判断
            /*Current_Height=telehwp.Elevator.getCurrentPosition();
            Current_Turn=telehwp.Base.getCurrentPosition();
            if(Current_Height>Safe_Height)
                IsTurnningOk=true;*/
            // TODO: 2021/12/1 转向判断

            //转向
            telehwp.Base.setPower(gamepad2.left_stick_x*0.3);

             //抬升
            telehwp.Elevator.setPower(gamepad2.right_stick_y);


            //夹子
            if(isOpen&&gamepad2.a){
                telehwp.Claw.setPosition(0.3);
                isOpen=false;
            }
            if(!isOpen&&gamepad2.b){
                telehwp.Claw.setPosition(0.05);
                isOpen=true;
            }

            //转盘
            while (gamepad2.b){
                telehwp.Rolling.setPower(1);
            }
            telehwp.Rolling.setPower(0);
            //reset
            telemetry.addData("IsTurnningok",IsTurnningOk);
            telemetry.addData("Isopen",isOpen);
            telemetry.addData("CurrentPosition",Current_Height);
            telemetry.addData("CurrentTurn",Current_Turn);
            telemetry.update();
        }
    }
}
