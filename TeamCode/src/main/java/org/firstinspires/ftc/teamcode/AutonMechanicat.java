package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous (name = "AutonMechanicat",group = "Mechanicats")

        public class AutonMechanicat extends LinearOpMode{
        //set motors
        private DcMotor frontRight = null;
        private DcMotor frontLeft = null;
        private DcMotor backRight = null;
        private DcMotor backLeft = null;
        private DcMotorEx Arm_Motor = null;

        public boolean left = false;
        public boolean blue = true;
        public int slots;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };



        private Servo Claw = null;
        private Servo Claw_2 = null;


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
        final public int max_arm_velo = 2000;

    final public int BOTTOM_ARM_POS = 200;
    //TODO get encoder value
    final public int LOW_ARM_POS =  1925; // = 13 inches

    //TODO get encoder value
    final public int MEDIUM_ARM_POS = 3147 ; // = 23 inches

    final public int TOP_ARM_POS = 4043; // = 33 inches

        //Vision
        private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
        private static final String VUFORIA_KEY =
                "AebftHb/////AAABmd/Ha8apiEpSpbtB4aKGLN9QTt3cxCjgODwhU7ZA4aNckDj/EDSRMUC5CYXZOvblT0jsfXOuQCJ5EV6vELtqHC0XbdmzxaabtF1/4QM6eykLYi+JLJe8tLT2YtkBAR2CDOK58E7IpUT5cr22fvYBv3inUpMIShMpeRVN595ZGrkQqjdrOoLG61yXez30PlEFaOzY2x5bRojrROJAUvca7TsUsLBE/33Q3ujLXKQWcuSNKiLZhKlS0b4EiH9kLd8/W936mBmTF2XUpXCQdEpPaqdOt3zJL/qrh39VUPRYdRvwthNfy0/9u1xklcpui940V3EOkEi0fqL3siQq/YKvCc+f3x1sUvEBHGUSRGxgafSN";
        private TFObjectDetector tfod;
        private VuforiaLocalizer vuforia;

    public void initialize()
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        Claw = hardwareMap.get(Servo.class,"Claw");
        Claw_2 =hardwareMap.get(Servo.class,"Claw2");

        Claw.setPosition(1);
        Claw_2.setPosition(0);

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

        Arm_Motor.setTargetPosition(BOTTOM_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);

        initVuforia();
        initTfod();


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public void liftArmGoMedium()
    {
        Arm_Motor.setTargetPosition(MEDIUM_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
    }

    public void claw_drop()
    {
        Claw.setPosition(0);
        Claw_2.setPosition(1);
    }

    public int marker_identification()
    {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        Recognition recognition = updatedRecognitions.get(0);

        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

        telemetry.addData(""," ");
        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
        if(recognition.getLabel() == "panel")
        {
            slots  =3;
        }
        if(recognition.getLabel() == "bulb")
        {
            slots  =2;
        }
        if(recognition.getLabel() == "bolt")
        {
            slots  =1;
        }
        return slots;
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
               // marker_identification();
            }

            telemetry.addData("Mode", "running");
            telemetry.update();




            if(slots == 2)
            {
                AutogoForwardSlot();
            }
            else if(slots == 1)
            {
                AutogoLeftSlot();
            }
            else
            {
                AutogoRightSlot();
            }


          /*
          else if (blue)
            {
                if(left)
                {
                    blueLeft();
                }
                else
                {
                    blueRight();
                }
            }
            else
            {
                if(left)
                {
                    redLeft();
                }
                else
                {
                    redRight();
                }
            } */
        }





        private void redLeft(){
            driveHeading(20,0,.7);
            telemetry.addData("Mode", "doing auton");
            telemetry.update();
            driveHeading(8,90,.7);
            liftArmGoMedium();
            driveHeading(4,0,.7);
            claw_drop();

        }

        private void redRight(){
            driveHeading(20,0,.7);
            driveHeading(8,-90,.7);
            driveHeading(4,0,.7);
            liftArmGoMedium();
            claw_drop();
        }

        private void blueLeft(){
            driveHeading(20,0,.7);
            driveHeading(8,-90,.7);
            liftArmGoMedium();
            driveHeading(4,0,.7);
            claw_drop();
        }

        private void blueRight(){
            driveHeading(20,0,.7);
            driveHeading(8,90,.7);
            liftArmGoMedium();
            driveHeading(4,0,.7);
            claw_drop();
        }
        private void AutogoForwardSlot()
        {
            driveHeading(20,0,.7);
        }
         private void AutogoLeftSlot()
        {
            left(20,.7);
            driveHeading(20,0,.7);
        }
        private void AutogoRightSlot()
        {
            right(20,.7);
            driveHeading(20,0,.7);
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }






        private void right (int distanceInInches,double speed){

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

        private void left (int distanceInInches,double speed){

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

