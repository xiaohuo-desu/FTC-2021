package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "Redside_Left_Auto")
public class AutoRedsideLeft extends LinearOpMode {

    OpenCvWebcam WebCam;

    Hardwaremap autohwp = new Hardwaremap();

    int distance;

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1000;

    ElapsedTime runtime = new ElapsedTime();

    private enum touch{
        Put,Catch
    }

    private enum level{
        level1,level2,level3
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CubeDetector detector = new CubeDetector(telemetry);
        WebCam.setPipeline(detector);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                WebCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Status","camera is not working");
            }
        });

        autohwp.Claw.setPosition(0.6);
        autohwp.Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        //判断
        switch (detector.getLocation()) {
            case LEFT:
                detectPosition=-2100;
                distance=22;
                break;
            case RIGHT:
                detectPosition=-800;
                distance=21;
                break;
            case MIDDLE:
                detectPosition=-1400;
                distance=22;
                break;
        }
        WebCam.stopStreaming();
        // +左前，+右前，+右后，+左后为前行
        encoderDrive_PlusElevator(DRIVE_SPEED,-24,24,-24,24,detectPosition,6);//左平移
        encoderDrive(1200,distance,distance,distance,distance,5);//前进
        sleep(200);
        autohwp.Claw.setPosition(0.45);
        sleep(200);
        //放（第一次）
        encoderDrive(DRIVE_SPEED,-5,-5,-5,-5,5);
        encoderDrive_PlusElevator(DRIVE_SPEED,-16,-16,-16,-16,0,6);//后退
        encoderDrive(DRIVE_SPEED,46,-46,46,-46,5);//左平移
        //转转盘
        PanelRotation();
        encoderDrive(DRIVE_SPEED,22,22,22,22,5);
    }

    private void encoderDrive(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches, double timeoutS) {
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

    private void encoderDrive_PlusElevator(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches,int ElevatorPosition,double timeoutS) {
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
            while (opModeIsActive() && ((autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy())||autohwp.Elevator.isBusy())&&runtime.seconds() < timeoutS) {
                // Display it for the driver.
                boolean IsRun=autohwp.Leftfront.isBusy()&& autohwp.Leftback.isBusy() && autohwp.Rightback.isBusy() && autohwp.Rightfront.isBusy();
                boolean IsReach=autohwp.Elevator.isBusy();
                if (!autohwp.Leftfront.isBusy()&& !autohwp.Leftback.isBusy() && !autohwp.Rightback.isBusy() && !autohwp.Rightfront.isBusy()) {
                    autohwp.Leftfront.setPower(0);
                    autohwp.Leftback.setPower(0);
                    autohwp.Rightfront.setPower(0);
                    autohwp.Rightback.setPower(0);
                }
                runtime.time();
                telemetry.addData("Path1", autohwp.Leftfront.getTargetPosition());
                telemetry.addData("Path2", autohwp.Leftfront.getCurrentPosition());
                telemetry.addData("IsRun",IsRun);
                telemetry.addData("IsReach",IsReach);
                telemetry.update();
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
    /*public void returnPosition(level detectlevel){
        switch (detectlevel){
            case level1:
                detectPosition=-800;
                break;
            case level2:
                detectPosition=-1400;
                break;
            case level3:
                detectPosition=-2100;
                break;
        }
    }*/

    public void PanelRotation(){
        autohwp.Rolling.setPower(-0.5);
        sleep(5000);
    }
}

