package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.List;
@Autonomous(name = "Robotivity Main Auto", group = ("Robotivity"))
public class Robotivity_Auto extends LinearOpMode {

    //set up motors
    private DcMotor motorFR= null;
    private DcMotor motorFL= null;
    private DcMotor motorBR= null;
    private DcMotor motorBL= null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private Servo grabber1 = null;
    private Servo grabber2 = null;
    private Servo blockServo = null;

    //gyro IMU
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    private int robotHeading = 0;


    private double pi = 3.1415;
   // private double ticksPerRev = 288;//rev
    private double ticksPerRev = 1120;//360; //40:1 motors
    private double wheelDiameter = 10;//in cm
    private double DistancePerTick = (pi *wheelDiameter) / ticksPerRev;
    private int driveBaseMotors = 4;
    private double fudge = 1.07;

    //vision
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AS5Paxn/////AAABmTmNqZt0CU5ntQbbqJokc6hp3fUR41J7N81dn/IigaWyHyinI3UoFokjKYYwayLnq+NEtZ7F5XRY7FFYe1EC5eST+kjxT+xSMAsmHMNVtcINyIjiO3/RZKWiQjwn1vVw6ygz7pdw+zKMQGQA9b8Yh+gv7uE3lSms5sM9qwMULyGT+ntR44ESkOBJl+XMk7X9Bxfv8maBSsaCztqChOghJ06hT3bmJ1f5D0jQ35CX0S+xoU8bJi/yQO8QdefYO509qv05tx0kNP3i2wPK8QexncHhjRYr0z+t2+XG7/eQLNmOxJCtnpbtEn/Jp99KfSCrFFXLSPxiAYbrktcc0TpPa+8M3BhnOc9a5TkEZQN2DgE9";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        motorBL = hardwareMap.get(DcMotor.class,"BLMotor");
        motorFL = hardwareMap.get(DcMotor.class,"FLMotor");
        motorBR = hardwareMap.get(DcMotor.class,"BRMotor");
        motorFR = hardwareMap.get(DcMotor.class,"FRMotor");
        rightIntake = hardwareMap.get(DcMotor.class,"Right Intake");
        leftIntake = hardwareMap.get(DcMotor.class,"Left Intake");
        grabber1 = hardwareMap.get(Servo.class,"Grabber 1");
        grabber2 = hardwareMap.get(Servo.class,"Grabber 2");
        blockServo = hardwareMap.get(Servo.class,"Block");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addLine("Motors done");
        telemetry.update();

        resetEncoders();

        blockServo.setPosition(.5);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addLine("Gyro Done");
        telemetry.update();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addLine("everything done");
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();


        //redOneQuarry();
        //redTwoBuildSite();
        blueOneQuarry();
        //blueTwoBuildsite();

    }

    private void redOneQuarry(){
        grabSkystoneRed();
        //crossBridgeRed();
        //dropSkystoneRed();
        //redOnePark();
    }

    private void redTwoBuildSite(){
        grabRedFoundation();
        redTwoPark();
    }

    private void blueOneQuarry(){
        //done?
        driveHeading(-19,0,1);
        right(16,.6);
        vision();
        sleep(2000);
        CameraDevice.getInstance().stop();
        grabSkyStoneBlue();
        blueOnePark();
    }

    private void blueTwoBuildsite(){
       grabBlueFoundation();
        blueTwoPark();
    }

    private void grabSkystoneRed() {

        if(vision() == 2){

        }
        if(vision() == 3){

        }
        if(vision() == 1){
            
        }
    }

    private void crossBridgeRed() {

        right(63,1);

    }

    private void dropSkystoneRed() {

    }

    private void redOnePark() {

        left(25,1);
    }

    private void grabRedFoundation(){
        left(27 ,1);
        //driveForward(14,.5);
        sleep(1000);
        right(120,1);//needs to go longer, no power to move foundation
    }

    private void redTwoPark(){

        sleep(100);
        //driveBackward(38,1);
    }

    private void grabBlueFoundation(){
        left(32,1);
        //driveBackward(7,.5);
        sleep(1000);
        right(35,1);
    }

    private void blueTwoPark(){
        sleep(100);
        //driveForward(35,1);

    }

    private void blueOnePark(){
        left(10,1);
        //rotate(-10,1);
        //driveHeading(-10,0,1);
    }

    private void grabSkyStoneBlue(){
        //this test to see if the robot can properly detect the blocks, will change when it can
            if (vision() == 1) { //left
                telemetry.addLine("vision = 1");
                telemetry.update();
                driveHeading(7,0,.5);
                rotate(74,.5);
                driveHeading(-12,0,.8);
                blockServo.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(75,1);
                blockServo.setPosition(.5);

            }
            if (vision() == 2) { //middle
                telemetry.addLine("vision = 2");
                telemetry.update();
                rotate(76,.5);
                driveHeading(-12,0,.8);
                blockServo.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(55,1);
                blockServo.setPosition(.5);

            }
            if (vision() == 3) { //right
                telemetry.addLine("vision = 3");
                telemetry.update();
                rotate(76,.5);
                left(3,1);
                driveHeading(-12,0,.8);
                blockServo.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(55,1);
                blockServo.setPosition(.5);
            }

    }

  /*  private void driveForward(int distanceInInches, double speed){

       int currentPosition = 0;
       int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

       telemetry.addData("Target Encoder: ", wantedPosition);
       resetEncoders();

       while ((currentPosition < wantedPosition) && opModeIsActive()){

           telemetry.addData("value of current position", currentPosition);
           telemetry.update();

           motorFL.setPower(speed);
           motorBL.setPower(speed);
           motorFR.setPower(speed);
           motorBR.setPower(speed);
           currentPosition = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition()
                   + motorBR.getCurrentPosition() +
                   motorBL.getCurrentPosition()) / 4;
           telemetry.update();
       }

       hardStop();

   }

    private void driveBackward(int distanceInInches,double speed){

        int currentPosition = 0;//(motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;
        int wantedPosition = (int) ((-distanceInInches / DistancePerTick)* fudge);

        resetEncoders();

        while ((currentPosition > wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.addData("wanted position",wantedPosition);
            telemetry.update();

            motorFL.setPower(-speed);
            motorBL.setPower(-speed);
            motorFR.setPower(-speed);
            motorBR.setPower(-speed);
            currentPosition = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;
        }

        hardStop();

    }*/

  /* private void turnLeft(int turnDistance, double speed){

        int currentPosition = (motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 2;
        int wantedPosition = (int) (turnDistance / DistancePerTick);

        resetEncoders();

        while ((currentPosition < wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.update();

            motorFL.setPower(-speed);
            motorBL.setPower(-speed);
            motorFR.setPower(speed);
            motorBR.setPower(speed);
            currentPosition = (motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 2;
        }

        hardStop();

    }

   private void turnRight(int turnDistance, double speed){

        int currentPosition = (motorFL.getCurrentPosition() + motorBL.getCurrentPosition()) / 2;
        int wantedPosition = (int) (turnDistance / DistancePerTick);

        resetEncoders();

        while ((currentPosition < wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.update();

            motorFL.setPower(speed);
            motorBL.setPower(speed);
            motorFR.setPower(-speed);
            motorBR.setPower(-speed);
            currentPosition = (motorFL.getCurrentPosition() + motorBL.getCurrentPosition()) / 2;
        }

        hardStop();
    }*/

  /* private void sideHeading(int distanceInInches, int targetHeading, double speed){

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

           //Apply the power settings to the motors
           motorFL.setPower(leftSpeed);
           motorBL.setPower(leftSpeed);
           motorFR.setPower(rightSpeed);
           motorBR.setPower(rightSpeed);

           //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
           distanceTraveled = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

       }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

       //Done so turn off the motors
       motorFL.setPower(0);
       motorBL.setPower(0);
       motorFR.setPower(0);
       motorBR.setPower(0);

       //Update the direction we think we are pointing
       robotHeading = targetHeading;
   }*/

    private void right (int distanceInInches,double speed){

        int currentPosition = 0;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

        resetEncoders();

        while ((currentPosition < wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.update();

            motorFL.setPower(speed);
            motorBL.setPower(-speed);
            motorFR.setPower(-speed);
            motorBR.setPower(speed);
            currentPosition = (motorFL.getCurrentPosition() + motorBR.getCurrentPosition()) / 2;
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

            motorFL.setPower(-speed);
            motorBL.setPower(speed);
            motorFR.setPower(speed);
            motorBR.setPower(-speed);
            currentPosition = (motorFR.getCurrentPosition() + motorBL.getCurrentPosition()) / 2;
        }

        hardStop();

    }

  /* private int GetTrueAngle(){
        float currAngle = GetAngle()*-1;

        if(currAngle < 0){
            currAngle = 360 + currAngle; // this code converst - angles to true north 0 to 359 angles
        }
        return (int)(currAngle+.5)%360;
    }*/

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
        motorBL.setPower(leftPower);
        motorFL.setPower(leftPower);
        motorFR.setPower(rightPower);
        motorBR.setPower(rightPower);

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
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

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

            //Apply the power settings to the motors
            motorFL.setPower(leftSpeed);
            motorBL.setPower(leftSpeed);
            motorFR.setPower(rightSpeed);
            motorBR.setPower(rightSpeed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

        }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        telemetry.addLine("before params creation");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addLine("after webcame mapping");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addLine("after classfactory instance creation");
        telemetry.update();

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private int vision() {

        float block1Left;
        float block2left;
        boolean skyStone1 = false;
        boolean skyStone2 = false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()>=2) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                Recognition recognition = updatedRecognitions.get(0);
                block1Left = recognition.getLeft();
                telemetry.addData("object 0 left pos:", block1Left);

                if (recognition.getLabel() == "Skystone") {
                    skyStone1 = true;
                }
                telemetry.addData("object 0 SS? ", skyStone1);

                recognition = updatedRecognitions.get(1);
                block2left = recognition.getLeft();
                telemetry.addData("object 1 left pos:", block2left);


                if (recognition.getLabel() == "Skystone")
                    skyStone2 = true;

                telemetry.addData("object 1 SS? ", skyStone2);
                telemetry.update();

                if (skyStone1) {
                    if (block1Left < block2left){
                        telemetry.addLine("CONFIRM SKYSTONE MIDDLE");
                        telemetry.update();
                        return 2;
                    }
                    else {
                        telemetry.addLine("CONFIRM SKYSTONE RIGHT");
                        telemetry.update();
                        return 3;
                    }
                } else if (skyStone2) {
                    if (block2left < block1Left) {
                        telemetry.addLine("CONFIRM SKYSTONE MIDDLE");
                        telemetry.update();
                        return 2;
                    }
                    else {
                        telemetry.addLine("CONFIRM SKYSTONE RIGHT");
                        telemetry.update();
                        return 3;
                    }
                }
                telemetry.addLine("CONFIRM SKYSTONE LEFT");
                telemetry.update();
                return 1;

            } else return -1;
        }
        return -1;
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

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    private void resetEncoders(){
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}