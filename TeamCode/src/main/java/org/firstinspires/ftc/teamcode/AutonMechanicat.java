package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

@Autonomous (name = "AutonMechanicat",group = "Mechanicats")
public class AutonMechanicat extends LinearOpMode{
    //set motors                                               {
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;
    private Servo outtake = null;
    private Servo orange = null;
    private Servo blue = null;
    private Servo Stonegrabber = null;

    //gyro IMU
        BNO055IMU imu;
        Orientation             lastAngles = new Orientation();
        double                  globalAngle, power = .30, correction;
        private int robotHeading = 0;

    private double pi = Math.PI;
        // private double ticksPerRev = 288;//rev
    private double ticksPerRev = 1120;
    private double wheelDiameter = 10;
    private double DistancePerTick = (pi *wheelDiameter) / ticksPerRev;
    private double orangeOpen = .5;
    private double blueOpen = .5;
    private double blueClose = -1.00;
    private double orangeClose = -1.00;
    private int driveBaseMotors = 4;
    private double fudge = 1.07;

     //Vision
        private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
        private static final String LABEL_FIRST_ELEMENT = "Stone";
        private static final String LABEL_SECOND_ELEMENT = "Skystone";
        private static final String VUFORIA_KEY =
                "AebftHb/////AAABmd/Ha8apiEpSpbtB4aKGLN9QTt3cxCjgODwhU7ZA4aNckDj/EDSRMUC5CYXZOvblT0jsfXOuQCJ5EV6vELtqHC0XbdmzxaabtF1/4QM6eykLYi+JLJe8tLT2YtkBAR2CDOK58E7IpUT5cr22fvYBv3inUpMIShMpeRVN595ZGrkQqjdrOoLG61yXez30PlEFaOzY2x5bRojrROJAUvca7TsUsLBE/33Q3ujLXKQWcuSNKiLZhKlS0b4EiH9kLd8/W936mBmTF2XUpXCQdEpPaqdOt3zJL/qrh39VUPRYdRvwthNfy0/9u1xklcpui940V3EOkEi0fqL3siQq/YKvCc+f3x1sUvEBHGUSRGxgafSN";
        private TFObjectDetector tfod;
        private VuforiaLocalizer vuforia;

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.addData("Mode", "calibrating...");
            telemetry.update();
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
            intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
            outtake = hardwareMap.get(Servo.class, "outtake");
            outtake = hardwareMap.get(Servo.class, "orange");
            outtake = hardwareMap.get(Servo.class, "blue");
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            driveHeading(108,0,1);


            while (!isStopRequested() && !imu.isGyroCalibrated())
            {
                sleep(50);
                idle();
            }

            /*while (!isStopRequested() && !imu.isGyroCalibrated()) {
                //sleep(20000);
                frontRight.setPower(1);
                frontLeft.setPower(1);
                backLeft.setPower(1);,
                backRight.setPower(1);
                sleep(5);
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            */
            /*
            telemetry.addData("Mode", "calibrating...");
            telemetry.update();
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
            intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
            outtake = hardwareMap.get(Servo.class, "outtake");
            outtake = hardwareMap.get(Servo.class, "orange");
            outtake = hardwareMap.get(Servo.class, "blue");
            imu = hardwareMap.get(BNO055IMU.class, "imu");


            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD); //
            intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addLine("Motors done");
            telemetry.update();

            resetEncoders();

            Stonegrabber.setPosition(.5);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            imu.initialize(parameters);


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

            // **********************************************************************************************************
            waitForStart();

            telemetry.addData("Mode", "running");
            telemetry.update();


            //redOneQuarry();
            //redTwoBuildSite();
            //blueOneQuarry();
            //blueTwoBuildsite();
            parkOnLine();
*/
        }

        private void parkOnLine() {
            left(20, 1);
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
            // move bot into camera position
            driveHeading(-19,0,1);
            right(16,.6);

            // figures out which block
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

        /* this is the code that determines where the stone is and then drives to it grabs it
                and then takes it over the center line                                          */

        private void grabSkyStoneBlue(){
            //this test to see if the robot can properly detect the blocks, will change when it can
            if (vision() == 1) { //left
                telemetry.addLine("vision = 1");
                telemetry.update();
                driveHeading(7,0,.5);
                rotate(74,.5);
                driveHeading(-12,0,.8);
                Stonegrabber.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(75,1);
                Stonegrabber.setPosition(.5);

            }
            if (vision() == 2) { //middle
                telemetry.addLine("vision = 2");
                telemetry.update();
                rotate(76,.5);
                driveHeading(-12,0,.8);
                Stonegrabber.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(55,1);
                Stonegrabber.setPosition(.5);

            }
            if (vision() == 3) { //right
                telemetry.addLine("vision = 3");
                telemetry.update();
                rotate(76,.5);
                left(3,1);
                driveHeading(-12,0,.8);
                Stonegrabber.setPosition(-.6);
                sleep(500);
                driveHeading(8,0,1);
                right(55,1);
                Stonegrabber.setPosition(.5);
            }

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

                //Apply the power settings to the motors
                frontLeft.setPower(leftSpeed);
                backLeft.setPower(leftSpeed);
                frontRight.setPower(rightSpeed);
                backRight.setPower(rightSpeed);

                //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
                distanceTraveled = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

            }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

            //Done so turn off the motors
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

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





















