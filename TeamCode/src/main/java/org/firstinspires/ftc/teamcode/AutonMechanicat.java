package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

@Autonomous (name = "AutonMechanicat",group = "Mechanicats")

        public class AutonMechanicat extends LinearOpMode{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    //set motors
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotorEx Arm_Motor = null;
    /// IMPORTANT STUFF AHEAD ///
    /// CHANGE BASED ON WHICH ROBOT ///
        final public int Robot = 2;    //1 = GERALD: 2 = REGINALD:

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int LastSeen = 0;

    // UNITS ARE METERS
    double tagsize = 0.166;
   private int Slot1 = 0;
   private int Slot2 = 0;
   private int Slot3 = 0;
   private int activeSlot = 2;
   public boolean left = false;
   public boolean blue = true;
   public int slots;

   private Servo Claw = null;
   private Servo Claw_2 = null;

   private ElapsedTime runtime = new ElapsedTime();

   //gyro IMU
   BNO055IMU imu;
   Orientation lastAngles = new Orientation();
   double globalAngle= 0;
   private int robotHeading = 0;
    private double pi = Math.PI;
    private double ticksPerRev = 428;
    private double wheelDiameter = 3;
    private double DistancePerTick = (pi *wheelDiameter) / ticksPerRev;
    private int driveBaseMotors = 4;
    private double fudge = 1.07;



    final public int max_velo = 1000;
    final public int max_arm_velo =4000;

    final public int BOTTOM_ARM_POS = 500;
    //TODO get encoder value
    final public int LOW_ARM_POS =  1925; // = 13 inches

    //TODO get encoder value
    final public int MEDIUM_ARM_POS = 3147 ; // = 23 inches

    final public int TOP_ARM_POS = 4043; // = 33 inches

    final public int AUTO_ARM_POS = 200;

        //Vision


    public void initialize()
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        Claw = hardwareMap.get(Servo.class,"Claw");
        Claw_2 =hardwareMap.get(Servo.class,"Claw2");


        Arm_Motor = hardwareMap.get(DcMotorEx.class,"Arm");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        InitVision();
    }

    public void liftArmGoGround()
    {
        Arm_Motor.setTargetPosition(0);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }
    public void liftArmGoGroundJunction()
    {
        Arm_Motor.setTargetPosition(BOTTOM_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }
    public void liftArmGoMedium()
    {
        Arm_Motor.setTargetPosition(MEDIUM_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }
    public void liftArmGoHigh()
    {
        Arm_Motor.setTargetPosition(TOP_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }

    public void liftArmGoAuto()
    {
        Arm_Motor.setTargetPosition(AUTO_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }

    public void claw_drop()
    {
        Claw.setPosition(.2);
        Claw_2.setPosition(.7);
    }

    public void claw_grab()
    {
        Claw.setPosition(.7);
        Claw_2.setPosition(.2);
    }

        private void InitVision()
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
    }

    private void ScanTags()
    {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                LastSeen = tag.id;
                if(tag.id == 1) //Tags are standard specified tag IDs. Robocats are using 0, 1 and 2
                {
                    Slot1++;
                    break;
                }
                else if(tag.id == 2) //Tags are standard specified tag IDs. Robocats are using 0, 1 and 2
                {
                    Slot2++;
                    break;
                }
                else if(tag.id == 3) //Tags are standard specified tag IDs. Robocats are using 0, 1 and 2
                {
                    Slot3++;
                    break;
                }
            }

        }
    }

    private void FindActiveSlot()
    {
        while(opModeIsActive() && (runtime.seconds() < 4)) //Scan for max time of 4 seconds
        {
            ScanTags();
            telemetry.addData("slot1",Slot1);
            telemetry.addData("slot2",Slot2);
            telemetry.addData("slot3",Slot3);


        }
        //NOTE : This ordering should return slot2 if they ar all the same
        //or if there are 2 the same including slot 2
        //Slot 2 is the easiest to navigate to is we are unsure
        if (Slot1 > Slot2)
        {
            if (Slot1 > Slot3){
                activeSlot = 1;
            }
            else
            {
                activeSlot = 3;
            }
        }
        else //slot 2 >= slot 1
        {
            if (Slot3 > Slot2){
                activeSlot = 3;
            }
            else
            {
                activeSlot = 2;
            }
        }

    }

    @Override
        public void runOpMode() {

            initialize();

            telemetry.addLine("everything done");
            telemetry.addData("Mode", "waiting for start");
            telemetry.update();

            // **********************************************************************************************************

            while(!isStarted())
            {
                ScanTags();
                telemetry.addData("slot1",Slot1);
                telemetry.addData("slot2",Slot2);
                telemetry.addData("slot3",Slot3);
                if(Robot == 1) {
                    telemetry.addLine("Robot is gerald");
                }
                else
                {
                    telemetry.addLine("Robot is reginald");
                }
                telemetry.update();

                telemetry.addData("last seen",LastSeen);
                telemetry.update();

            }
            runtime.reset();

            //reset scanner
            Slot1 = 0;
            Slot2 = 0;
            Slot3 = 0;

            Claw.setPosition(1);
            Claw_2.setPosition(0);
            sleep(2000);
            //liftArmGoGroundJunction();
            liftArmGoAuto();
            FindActiveSlot();   //should wait 2 sec until runtime is 4 sec
            telemetry.addData("Mode", "running");
            telemetry.update();

            if(activeSlot == 2)
            {
                AutogoForwardSlot();

            }
            else if(activeSlot == 3)
            {
                AutogoRightSlot();
            }
            else
            {
                AutogoLeftSlot();
            }

            sleep(2000);
            liftArmGoGround();
            sleep(2000);    //time to let all functions finish


    }
        private void AutogoForwardSlot()
        {
            driveHeading(22,0,.7);
        }
         private void AutogoRightSlot()
        {
            if(Robot == 2){
                Right(18,.6);
            }
            else {
                Right(20,.6);
            }
            driveHeading(18,0,.7);
        }
        private void AutogoLeftSlot()
        {   if(Robot == 2) {
                Left(18,.6);
            }
            else{
                Left(20,.6);
            }
            driveHeading(18,0,.7);
        }











        private void Left(int distanceInInches, double speed){

            int currentPosition = 0;
            int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

            resetEncoders();

            while ((currentPosition < wantedPosition) && opModeIsActive()){
                telemetry.addData("value of current position", currentPosition);
                telemetry.update();

                frontRight.setPower(-speed);
                frontLeft.setPower(speed);
                backRight.setPower(speed);
                backLeft.setPower(-speed);
                currentPosition = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
            }

            hardStop();

        }

        private void Right(int distanceInInches, double speed){

            int currentPosition = 0;
            int wantedPosition = (int) ((distanceInInches / DistancePerTick) * fudge);

            resetEncoders();

            while ((currentPosition < wantedPosition) && opModeIsActive()){
                telemetry.addData("value of current position", currentPosition);
                telemetry.update();

                frontRight.setPower(speed);
                frontLeft.setPower(-speed);
                backRight.setPower(-speed);
                backLeft.setPower(speed);
                currentPosition = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
            }

            hardStop();

        }

        private void rotate(int degrees, double power)
        {
            double  leftPower, rightPower;

            // restart imu movement tracking.
            resetAngle();

            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            if (degrees < 0)
            {   // turn right.
                leftPower = power;
                rightPower = -power;
            }
            else if (degrees > 0)
            {   // turn left.
                leftPower = -power;
                rightPower = power;
            }
            else return;

            // set power to rotate.
            backLeft.setPower(leftPower);
            frontLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            // rotate until turn is completed.
            if (degrees < 0)
            {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {}

                while (opModeIsActive() && getAngle() > degrees) {}
            }
            else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {}

            // turn the motors off.
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // wait for rotation to stop.
            sleep(1000);

            // reset angle tracking on new heading.
            resetAngle();
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

        public void driveHeading(int distanceInInches, int targetHeading, double speed)
        {
            int error;
            double currentHeading;
            double speedCorrection = 0;
            double leftSpeed;
            double rightSpeed;
            double gain = .06;
            int distanceTraveled;
            int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

            resetEncoders();

            if (wantedPosition > 0)
            {
                //Going forwards so make sure the speed is +ve
                speed = Math.abs(speed);
            }
            else
            {
                //Going backwards so make sure the speed is -ve
                speed = -Math.abs(speed);
            }


            do {
                //Find where we are currently pointing
                currentHeading = getAngle();

                //Calculate how far off we are
                error = (int)(currentHeading - targetHeading);

                //Using the error calculate some correction factor
                speedCorrection = error * gain;

                //Adjust the left and right power to try and compensate for the error
                leftSpeed = speed + speedCorrection;
                rightSpeed = speed - speedCorrection;

                leftSpeed= leftSpeed/2;
                rightSpeed= rightSpeed/2;

                //Apply the power settings to the motors
                frontLeft.setPower(leftSpeed);
                backLeft.setPower(leftSpeed);
                frontRight.setPower(rightSpeed);
                backRight.setPower(rightSpeed);

                //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
                distanceTraveled = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / driveBaseMotors;


            }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

            //Done so turn off the motors
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            //Update the direction we think we are pointing
            robotHeading = targetHeading;
        }
    public void driveTranslation(int distanceInInches, int targetHeading, double speed)
    {
        int error;
        double currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;
        double gain = .06;
        int distanceTraveled;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

        resetEncoders();

        if (wantedPosition > 0)
        {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        }
        else
        {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }


        do {
            //Find where we are currently pointing
            currentHeading = getAngle();

            //Calculate how far off we are
            error = (int)(currentHeading - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speed + speedCorrection;
            rightSpeed = speed - speedCorrection;


            //Adjust the left and right power to try and compensate for the error

            leftSpeed= leftSpeed/2;
            rightSpeed= rightSpeed/2;

            //Apply the power settings to the motors
            frontLeft.setPower(leftSpeed);
            backLeft.setPower(leftSpeed);
            frontRight.setPower(rightSpeed);
            backRight.setPower(rightSpeed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / driveBaseMotors;


        }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }








        private void resetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }


        public float GetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return lastAngles.firstAngle;
        }

        private void hardStop(){

            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }

        private void resetEncoders() {
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

