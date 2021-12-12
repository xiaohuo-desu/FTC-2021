package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;

@Autonomous(name = "Redside_Right_Auto")
public class AutoRedsideRight extends LinearOpMode {

    Hardwaremap autohwp = new Hardwaremap();

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1200;
    static final double TURN_SPEED = 0.5;

    ElapsedTime runtime = new ElapsedTime();

    public enum touch{
        Put,Catch
    }

    public enum level{
        level1,level2,level3
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);
        autohwp.Claw.setPosition(0.3);
        autohwp.Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        // +左前，+右前，+右后，+左后为前行
        returnPosition(level.level3);
        encoderDrive_PlusElevator(DRIVE_SPEED,24,-24,24,-24,detectPosition);//左平移
        encoderDrive(1200,23,23,23,23,5);//前进
        sleep(200);
        ClawDrive(touch.Put);
        sleep(100);
        //倾倒（第一次）
        encoderDrive(DRIVE_SPEED,-8,-8,-8,-8,5);//后退
        encoderDrive(DRIVE_SPEED,-15,15,15,-15,5);//右转
        encoderDrive(DRIVE_SPEED,-19,19,-19,19,5);//右平移
        encoderDrive_PlusElevator(1500,46,46,46,46,0);//前行
        //夹取
        encoderDrive(500,5,5,5,5,5);//前行
        ClawDrive(touch.Catch);
        sleep(200);
        encoderDrive(300,-5,5,-5,5,5);
        sleep(200);
        encoderDrive(600,-15,-15,-15,-15,5);

        encoderDrive_PlusElevator(1000,-36,-36,-36,-36,-2100);//后退

        encoderDrive_PlusBase(2000,23,-23,23,-23,-4000);//左平移
        ClawDrive(touch.Put);
        sleep(200);

        encoderDrive_PlusBase(DRIVE_SPEED,-22,22,-22,21,0);//左平移
        encoderDrive(300,-3,3,-3,3,5);
        encoderDrive_PlusElevator(DRIVE_SPEED,48,48,48,48,0);//前行
//--------------------两次------------------------------//
        /*encoderDrive(800,4,4,4,4,5);//前行
        ClawDrive(touch.Catch);
        sleep(200);
        encoderDrive(300,-5,5,-5,5,5);
        sleep(200);
        encoderDrive(600,-10,-10,-10,-10,5);


        encoderDrive_PlusElevator(DRIVE_SPEED,-41,-41,-41,-41,-600);//后退

        encoderDrive_PlusBase(2000,24,-24,24,-24,-4000);//左平移
        ClawDrive(touch.Put);
        sleep(200);

        encoderDrive_PlusBase(DRIVE_SPEED,-23,23,-23,23,0);//左平移
        encoderDrive(300,-3,3,-3,3,5);
        encoderDrive_PlusElevator(2000,50,50,50,50,0);//前行*/
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

    boolean TurnningOk=false;

    public void ClawDrive(touch touchstate ){
        switch (touchstate){
            case Put:
                autohwp.Claw.setPosition(0.2);
                break;
            case Catch:
                autohwp.Claw.setPosition(0.4);
                break;
        }
    }

    public void encoderDrive_PlusBase(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches,int Baseposition) {
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
            autohwp.Base.setTargetPosition(Baseposition);

            // Turn On RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            autohwp.Leftfront.setVelocity(Math.abs(speed));
            autohwp.Rightfront.setVelocity(Math.abs(speed));
            autohwp.Leftback.setVelocity(Math.abs(speed));
            autohwp.Rightback.setVelocity(Math.abs(speed));
            autohwp.Base.setPower(1);
            while (opModeIsActive() && autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy()||autohwp.Base.isBusy()) {
                // Display it for the driver.
                if (!autohwp.Leftfront.isBusy()&& !autohwp.Leftback.isBusy() && !autohwp.Rightback.isBusy() && !autohwp.Rightfront.isBusy()) {
                    autohwp.Leftfront.setPower(0);
                    autohwp.Leftback.setPower(0);
                    autohwp.Rightfront.setPower(0);
                    autohwp.Rightback.setPower(0);
                }
                telemetry.addData("Path1", autohwp.Leftfront.getTargetPosition());
                telemetry.addData("Path2", autohwp.Leftfront.getCurrentPosition());
                telemetry.update();
            }
            while (autohwp.Base.isBusy()){

            }
            // Stop all motion;
            autohwp.Base.setPower(0);
            autohwp.Leftfront.setPower(0);
            autohwp.Leftback.setPower(0);
            autohwp.Rightfront.setPower(0);
            autohwp.Rightback.setPower(0);
            // Turn off RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(50);   // optional pause after each move
        }
    }

    public void encoderDrive_PlusElevator(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches,int ElevatorPosition) {
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
            autohwp.Elevator.setTargetPosition(ElevatorPosition);
            // Turn On RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            autohwp.Leftfront.setVelocity(Math.abs(speed));
            autohwp.Rightfront.setVelocity(Math.abs(speed));
            autohwp.Leftback.setVelocity(Math.abs(speed));
            autohwp.Rightback.setVelocity(Math.abs(speed));
            autohwp.Elevator.setPower(1);
            while (opModeIsActive() && autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy()||autohwp.Elevator.isBusy()) {
                // Display it for the driver.
                boolean IsRun=autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy();
                boolean IsReach=autohwp.Elevator.isBusy();
                if (!autohwp.Leftfront.isBusy()&& !autohwp.Leftback.isBusy() && !autohwp.Rightback.isBusy() && !autohwp.Rightfront.isBusy()) {
                    autohwp.Leftfront.setPower(0);
                    autohwp.Leftback.setPower(0);
                    autohwp.Rightfront.setPower(0);
                    autohwp.Rightback.setPower(0);
                }
                telemetry.addData("Path1", autohwp.Leftfront.getTargetPosition());
                telemetry.addData("Path2", autohwp.Leftfront.getCurrentPosition());
                telemetry.addData("IsRun",IsRun);
                telemetry.addData("IsReach",IsReach);
                telemetry.update();
            }
            while (autohwp.Elevator.isBusy()){

            }
            // Stop all motion;
            autohwp.Elevator.setPower(0);
            autohwp.Leftfront.setPower(0);
            autohwp.Leftback.setPower(0);
            autohwp.Rightfront.setPower(0);
            autohwp.Rightback.setPower(0);
            // Turn off RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(50);   // optional pause after each move
        }
    }

    int detectPosition=0;
    public void returnPosition(level detectlevel){
        switch (detectlevel){
            case level1:
                detectPosition=-600;
                break;
            case level2:
                detectPosition=-1000;
                break;
            case level3:
                detectPosition=-2100;
                break;
        }
        }
}


