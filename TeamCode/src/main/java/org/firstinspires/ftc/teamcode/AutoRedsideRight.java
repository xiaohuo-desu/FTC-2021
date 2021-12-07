package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;

@Autonomous(name = "Redside_Auto")
public class AutoRedsideRight extends LinearOpMode {

    Hardwaremap autohwp = new Hardwaremap();

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 2000;
    static final double TURN_SPEED = 0.5;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);
        waitForStart();
        ElevatorDrive(200,-600);
        sleep(200);
       ElevatorDrive(200,0 );
       /* // -左前，-右前，+右后，+左后为前行
        encoderDrive(DRIVE_SPEED,-21,21,-21,21,5);//右平移
        encoderDrive(DRIVE_SPEED,17,17,-17,-17,5);//后退
        //倾倒（第一次）
        encoderDrive(DRIVE_SPEED,-5,-5,5,5,5);//前行
        encoderDrive(DRIVE_SPEED,14,-14,-14,14,5);//右转
        encoderDrive(DRIVE_SPEED,-20,20,-20,20,5);//右平移
        encoderDrive(DRIVE_SPEED,-50,-50,50,50,5);//前行
        //吸取

        encoderDrive(100,-5,-5,5,5,5);//前行
        sleep(1000);

        encoderDrive(DRIVE_SPEED,56,56,-56,-56,5);//后退
        encoderDrive(DRIVE_SPEED,5,-5,5,-5,5);//左平移
        encoderDrive(DRIVE_SPEED,-13,13,13,-13,5);//右转
        encoderDrive(DRIVE_SPEED,-8,-8,8,8,5);
        encoderDrive(DRIVE_SPEED,17,17,-17,-17,5);//后退
        //倾倒（第二次）
        encoderDrive(DRIVE_SPEED,-5,-5,5,5,5);//前行
        encoderDrive(DRIVE_SPEED,14,-14,-14,14,5);//右转
        encoderDrive(DRIVE_SPEED,-20,20,-20,20,5);//右平移
        encoderDrive(DRIVE_SPEED,-55,-55,55,55,5);//前行
        //吸取

        encoderDrive(100,-5,-5,5,5,5);//前行
        sleep(1000);

        encoderDrive(DRIVE_SPEED,56,56,-56,-56,5);//后退
        encoderDrive(DRIVE_SPEED,5,-5,5,-5,5);//右平移
        encoderDrive(DRIVE_SPEED,-14,14,14,-14,5);//右转
        encoderDrive(DRIVE_SPEED,-8,-8,8,8,5);
        encoderDrive(DRIVE_SPEED,18,18,-18,-18,5);//后退
        //倾倒（第三次）
        encoderDrive(DRIVE_SPEED,-5,-5,5,5,5);//前行
        encoderDrive(DRIVE_SPEED,14,-14,-14,14,5);//右转
        encoderDrive(DRIVE_SPEED,-20,20,-20,20,5);//右平移
        encoderDrive(DRIVE_SPEED,-50,-50,50,50,5);//前行*/

    }

    public void encoderDrive(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches, double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = autohwp.Leftfront.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = autohwp.Leftback.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
            newRightfrontTarget = autohwp.Rightfront.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
            newRightbackTarget = autohwp.Rightback.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
            autohwp.Leftfront.setTargetPosition(newLeftfrontTarget);
            autohwp.Leftback.setTargetPosition(newLeftbackTarget);
            autohwp.Rightfront.setTargetPosition(newRightfrontTarget);
            autohwp.Rightback.setTargetPosition(newRightbackTarget);

            // Turn On RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            autohwp.Leftfront.setVelocity(Math.abs(speed));
            autohwp.Rightfront.setVelocity(Math.abs(speed));
            autohwp.Leftback.setVelocity(Math.abs(speed));
            autohwp.Rightback.setVelocity(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && /*(runtime.seconds() < timeoutS) &&*/ autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy()) {
                // Display it for the driver.
                telemetry.addData("Path1", autohwp.Leftfront.getTargetPosition());
                telemetry.addData("Path2", autohwp.Leftfront.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;

            autohwp.Leftfront.setPower(0);
            autohwp.Leftback.setPower(0);
            autohwp.Rightfront.setPower(0);
            autohwp.Rightback.setPower(0);
            // Turn off RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move
        }
    }


    public void BaseDrive(double speed,int targetPosition){
        autohwp.Base.setTargetPosition(targetPosition);
        autohwp.Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //if (TurnningOk)
        autohwp.Base.setPower(speed);
        while (autohwp.Base.isBusy()){
            telemetry.addData("CurrentPosition",autohwp.Base.getCurrentPosition());
            telemetry.addData("TargetPosition",autohwp.Base.getTargetPosition());
            telemetry.update();
        }
        autohwp.Base.setPower(0);
        autohwp.Base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    boolean TurnningOk=false;
    public void ElevatorDrive(double speed,int targetPosition){
        autohwp.Elevator.setTargetPosition(targetPosition);
        autohwp.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autohwp.Elevator.setVelocity(speed);
        if(autohwp.Elevator.getCurrentPosition()<-400){
            TurnningOk=true;
        }
        else {
            TurnningOk=false;
        }
        while (autohwp.Elevator.isBusy()){
            telemetry.addData("CurrentHeight",autohwp.Elevator.getCurrentPosition());
            telemetry.addData("TargetHeight",autohwp.Elevator.getTargetPosition());
            telemetry.update();
        }
        autohwp.Elevator.setPower(0);
        autohwp.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


