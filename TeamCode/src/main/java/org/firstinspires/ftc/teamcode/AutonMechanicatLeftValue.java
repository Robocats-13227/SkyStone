package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "AutonMechanicatLeftValue",group = "Mechanicats")
public class AutonMechanicatLeftValue extends LinearOpMode{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    //set motors
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx Arm_Motor = null;
    private Rev2mDistanceSensor Pole_Senor = null;
    /// IMPORTANT STUFF AHEAD ///
    /// CHANGE BASED ON WHICH ROBOT ///

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

    final public double ArmotorTickPerInch = 148.1;

    final public int BOTTOM_ARM_POS = 500;
    //TODO get encoder value
    final public int LOW_ARM_POS =  1925; // = 12 inches

    //TODO get encoder value
    final public int MEDIUM_ARM_POS = 3147 ; // = 23 inches

    final public int TOP_ARM_POS = 3943; // = 33 inches

    final public int AUTO_ARM_POS = 200;

    final public int ConeStackStartingPos = (int)(ArmotorTickPerInch*1.23)*4;


    public int ConeCount = 0;

    //Vision


    public void initialize()
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        Claw = hardwareMap.get(Servo.class,"Claw");
        Claw_2 =hardwareMap.get(Servo.class,"Claw2");

        Pole_Senor = hardwareMap.get(Rev2mDistanceSensor.class,"Pole_Sensor");


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
    public void liftArmGoLow()
    {
        Arm_Motor.setTargetPosition(LOW_ARM_POS);
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
        Claw.setPosition(0);
        Claw_2.setPosition(1);
    }

    public void claw_grab()
    {
        Claw.setPosition(1);
        Claw_2.setPosition(0);
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
        while(opModeIsActive() && (runtime.seconds() < .5)) //Scan for max time of 4 seconds
        {
            ScanTags();
            telemetry.addData("slot1",Slot1);
            telemetry.addData("slot2",Slot2);
            telemetry.addData("slot3",Slot3);
            telemetry.update();


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
    public void runAuto2()
    {
        telemetry.addLine("driving forward");
        telemetry.update();

        driveHeading(.5,0,.5);
        sleep(500);
        telemetry.addLine("moving to signal zone");
        telemetry.update();

        if(activeSlot == 2)
        {
            telemetry.addLine("driving towards 2 ");
            telemetry.update();
            driveHeading(20,0,0.5);

        }
        else if(activeSlot == 3)
        {
            telemetry.addLine("Strafing towards 3 ");
            telemetry.update();
            Strafe(20,.5,1);
            sleep(200);
            telemetry.addLine("driving towards 3 ");
            telemetry.update();
            driveHeading(20,0,.5);
        }
        else
        {
            telemetry.addLine("strafing towards 1 ");
            telemetry.update();
            Strafe(20,.5,-1);
            sleep(200);
            telemetry.addLine("driving towards 1 ");
            telemetry.update();
            driveHeading(20,0,.5);
        }

        liftArmGoGround();
        sleep(3000);    //time to let all functions finish

    }
    public void runAuto2High()
    {


        driveHeading(44, 0, 1);
        sleep(100);
        driveHeading(-7, 0, .7);
        sleep(100);
        Strafe(5,.5,-1);

        SeekAndDestroy(-1,.1,3,0,0);
        driveHeading(26.5,90 , .5);
        claw_grab();
        sleep(600);
        Arm_Motor.setTargetPosition(ConeStackStartingPos+((1+ConeCount)*(int)(ArmotorTickPerInch*4)));
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
        while (Arm_Motor.isBusy()){}
        driveHeading(-17,90,1);
        rotatetoTargetHeading(0,.3);
        SeekAndDestroy(-1,.1,3,0,0);
        sleep(100);
        if(activeSlot == 2) {
            AutogoMiddleSlot();
        } else if (activeSlot == 3) {
            AutogoRightSlot();
        } else {
            AutogoLeftSlot();
        }
    }
    public void runAuto3High()
    {
        driveHeading(44, 0, 1);
        sleep(100);
        driveHeading(-7.25, 0, .7);
        sleep(100);
        Strafe(5,.5,-1);

        SeekAndDestroy(-1,.175,3,0,0);
        int x = 1;
        while(x<3) {
            driveHeading(26.5, 90, .5);
            claw_grab();
            sleep(600);
            Arm_Motor.setTargetPosition(ConeStackStartingPos + ((x) * (int) (ArmotorTickPerInch * 3)));
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setVelocity(max_arm_velo);
            while (Arm_Motor.isBusy()) {
            }
            driveHeading(-17, 90, 1);
            rotatetoTargetHeading(0, 1);
            SeekAndDestroy(-1, .2, 3, 0,x);
            x++;
        }
        sleep(100);
        if(activeSlot == 2) {
            AutogoMiddleSlot();
        } else if (activeSlot == 3) {
            AutogoRightSlot();
        } else {
            AutogoLeftSlot();
        }
    }
    public void runAuto1High4low()
    {
        driveHeading(44, 0, 1);
        sleep(100);
        driveHeading(-7.25, 0, .7);
        sleep(100);
        Strafe(5,.5,-1);

        SeekAndDestroy(-1,.175,3,0,0);
        driveHeading(26.5, 90, .5);
        int x = 1;
        while(x<5) {
            claw_grab();
            sleep(500);
            Arm_Motor.setTargetPosition(ConeStackStartingPos + ((x) * (int) (ArmotorTickPerInch * 3)));
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setVelocity(max_arm_velo);
            while (Arm_Motor.isBusy()) {
            }
            driveHeading(-15, 90, 1);
            liftArmGoAuto();
            SeekAndDestroy(1, .2, 1, 0,x);
            Strafe(7,.5,-1);
            driveHeading(15, 90, 1);
            x++;
        }
        sleep(100);
        if(activeSlot == 2) {
            AutogoMiddleSlot();
        } else if (activeSlot == 3) {
            AutogoRightSlot();
        } else {
            AutogoLeftSlot();
        }
    }

    public void runAuto1(){



        driveHeading(44, 0, 1);
        sleep(100);
        driveHeading(-8, 0, 1);
        sleep(100);
        /*
       rotatetoTargetHeading(-45,.5);
        sleep(100);
        rotatetoTargetHeading(-45,.2);
         */
        Strafe(9.5,.3,1);
        liftArmGoHigh();
        sleep(1500);
        driveHeading(2.25, 0, .2);
        Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition()-(int)(3*ArmotorTickPerInch));
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
        claw_drop();
        sleep(100);
        liftArmGoHigh();
        sleep(1500);
        Arm_Motor.setTargetPosition(ConeStackStartingPos-(ConeCount*(int)(ArmotorTickPerInch*1.23)));
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
        driveHeading(-.75, 0, .5);
        rotatetoTargetHeading(90,.2);
        rotatetoTargetHeading(90,.2);
        driveHeading(26.5,90 , .5);
        claw_grab();
        sleep(250);
        Arm_Motor.setTargetPosition(ConeStackStartingPos+((1+ConeCount)*(int)(ArmotorTickPerInch*3.65)));
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
        sleep(100);
        driveHeading(-28,90 , .5);
        rotatetoTargetHeading(0,.5);
        sleep(100);
        rotatetoTargetHeading(0,.5);
        liftArmGoHigh();
        sleep(1000);
        driveHeading(2,0 , .5);
        Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition()-(int)(3*ArmotorTickPerInch));
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_arm_velo);
        sleep(200);
        claw_drop();
        sleep(500);
        driveHeading(-2.5, 0, .2);
        liftArmGoGround();
        rotatetoTargetHeading(90,.2);
        if (activeSlot == 2) {
            AutogoMiddleSlot();
        } else if (activeSlot == 3) {
            AutogoRightSlot();
        } else {
            AutogoLeftSlot();
        }




        /*
        while(ConeCount <2)
        {
            Arm_Motor.setTargetPosition(ConeStackStartingPos-(ConeCount*(int)(ArmotorTickPerInch*1.23)));
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setVelocity(max_arm_velo);
            claw_drop();
            sleep(200);
            driveHeading(42, -90, .3);
            claw_grab();
            sleep(200);
            Arm_Motor.setTargetPosition(ConeStackStartingPos+((1+ConeCount)*(int)(ArmotorTickPerInch*3.65)));
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setVelocity(max_arm_velo);
            driveHeading(-42, -90, .7);
            sleep(200);
            liftArmGoHigh();
            sleep(200);
            Strafe(10,.2,1);
            claw_drop();
            sleep(200);
            Strafe(10,.7,-1);
            ConeCount++;
        }
            if (activeSlot == 2) {
                AutogoForwardSlot();
            } else if (activeSlot == 3) {
                AutogoRightSlot();
            } else {
                AutogoLeftSlot();
            }
         */
        sleep(3000);    //time to let all functions finish

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

            ScanTags();   //should wait 2 sec until runtime is 4 sec
            runtime.reset();
            telemetry.addData("slot1",Slot1);
            telemetry.addData("slot2",Slot2);
            telemetry.addData("slot3",Slot3);


            telemetry.update();



            telemetry.addData("last seen",LastSeen);
        }
        //reset scanner
        Slot1 = 0;
        Slot2 = 0;
        Slot3 = 0;
        FindActiveSlot();



        Claw.setPosition(1);
        Claw_2.setPosition(0);
        sleep(700);
        //liftArmGoGroundJunction();
        liftArmGoAuto();
        telemetry.addData("Mode", "running");
        telemetry.addData("Active Slot",activeSlot);
        telemetry.update();


        //runAuto2High();
        runAuto3High();
        //runAuto1High4low();





    }


    private void AutogoRightSlot()
    {

        driveHeading(20,90,.7);
    }
    private void AutogoMiddleSlot()
    {

        driveHeading(8,90,.7);
    }
    private void AutogoLeftSlot()
    {

        driveHeading(-9,90,.7);
    }
    private void Test()
    {
        driveHeading(12,0,.5);
    }

    private double CalculateCorrectionPower(double error, int CorrectionMode ) {

        if(CorrectionMode == 0)
        {

            return error * .06;
        }
        else if(CorrectionMode == 1)
        {
            if(Math.abs(error) <2)
            {
                return 0;
            }
            else if(Math.abs(error) <10)
            {
                return error * .01;
            }
            else
            {
                return error*.09;
            }
        }
        else if(CorrectionMode == 2)
        {
            if(Math.abs(error) <2)
            {
                return 0;
            }
            else if(Math.abs(error) <10)
            {
                return Math.signum(error)*.3;
            }
            else
            {
                return Math.signum(error)*.8;
            }
        }
        else return 0;
    }
    private void Strafe(double distanceInInches, double speed, int LeftRight ){

        int currentPosition = 0;
        double currentHeading;
        double wantedPosition =  ((distanceInInches / DistancePerTick) * fudge);
        double targetHeading = getAngle();

        resetEncoders();

        while ((Math.abs(currentPosition) < wantedPosition) && opModeIsActive()){


            currentHeading = getAngle();

            //Calculate how far off we are
            double error = currentHeading - targetHeading;

            //Using the error calculate some correction factor
            double speedCorrection = CalculateCorrectionPower(error,0);


            double MotorSpeed1 = speed  ;
            double MotorSpeed2 = -speed  ;

            telemetry.addData("error",error);
            telemetry.addData("motor1",MotorSpeed1);
            telemetry.addData("motor2",MotorSpeed2);
            telemetry.addData("Speed correction",speedCorrection);
            telemetry.update();

            ControlMotors(((MotorSpeed2*LeftRight) - speedCorrection),
                    ((MotorSpeed1*LeftRight) + speedCorrection),
                    ((MotorSpeed1*LeftRight) - speedCorrection),
                    ((MotorSpeed2*LeftRight) + speedCorrection),
                    false,50);


            currentPosition = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
            currentHeading = getAngle();
        }

        hardStop();

    }
    private void SeekAndDestroy(int LeftRight, double speed,int pole , double targetHeading,int amount_of_cones)
    {
        int currentPosition = 0;
        double currentHeading;
        double Distance_backwards = 0;
        int check = 0;


        resetEncoders();
        double DistanceFromPole = Pole_Senor.getDistance(DistanceUnit.INCH);

        while ((DistanceFromPole > 10) && opModeIsActive()){


            currentHeading = getAngle();

            //Calculate how far off we are
            double error = currentHeading - targetHeading;

            //Using the error calculate some correction factor
            double speedCorrection = 0;// CalculateCorrectionPower(error,0);


            double MotorSpeed1 = speed ;
            double MotorSpeed2 = -speed ;

            telemetry.addData("error",error);
            telemetry.addData("motor1",MotorSpeed1);
            telemetry.addData("motor2",MotorSpeed2);
            telemetry.addData("check",check);
            telemetry.addLine("Speed correction");
            telemetry.update();

            ControlMotors(((MotorSpeed2*LeftRight)+speedCorrection ),
                    ((MotorSpeed1*LeftRight) - speedCorrection ),
                    ((MotorSpeed1*LeftRight) +speedCorrection),
                    ((MotorSpeed2*LeftRight) -speedCorrection),
                    false,150);

            DistanceFromPole = Pole_Senor.getDistance(DistanceUnit.INCH);
            /*
            if (DistanceFromPole < 11)
            {
                check++;
            }
            else
            {
                check = 0;
            }
             */
        }
        telemetry.addLine("found pole");
        telemetry.update();
        hardStop();

        sleep(100);
        double DistanceFromPole2 = Pole_Senor.getDistance(DistanceUnit.INCH);
        if (DistanceFromPole2 < 11)
        {
            DistanceFromPole = DistanceFromPole2;

        }
        if(pole ==1)
        {

            DistanceFromPole = Pole_Senor.getDistance(DistanceUnit.INCH);
            liftArmGoLow();
            Distance_backwards = 5.25;
        }
        else if(pole==2)
        {
            DistanceFromPole = Pole_Senor.getDistance(DistanceUnit.INCH);
            liftArmGoMedium();
            Distance_backwards = 5.25;
        }
        else if(pole == 3)
        {

            DistanceFromPole = Pole_Senor.getDistance(DistanceUnit.INCH);
            liftArmGoHigh();
            Distance_backwards = 5.5;

        }
        else
        {
            liftArmGoAuto();
        }


        telemetry.addData("Disance from pole",DistanceFromPole);
        telemetry.update();
        while (Arm_Motor.isBusy())
        {

        }


        driveHeading(DistanceFromPole-Distance_backwards,(int)getAngle(),.3);
        sleep(100);
        Arm_Motor.setTargetPosition(Arm_Motor.getCurrentPosition()-(int)(3*ArmotorTickPerInch));
        Arm_Motor.setVelocity(max_arm_velo);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm_Motor.isBusy())
        {

        }
        claw_drop();
        sleep(100);
        if(pole ==1)
        {
            liftArmGoLow();

        }
        else if(pole==2)
        {
            liftArmGoMedium();

        }
        else if(pole == 3)
        {
            liftArmGoHigh();

        }
        else
        {
            liftArmGoAuto();
        }
        driveHeading(-1,(int)getAngle(),.3);
        Arm_Motor.setTargetPosition((int)(ConeStackStartingPos-((ArmotorTickPerInch*1.23))*amount_of_cones));









        hardStop();


    }

    private void ControlMotors(double FrontLeft, double FrontRight, double BackLeft, double BackRight, boolean Velocity,int delay ) {

        double FRCurrPower = 0;
        double FLCurrPower = 0;
        double BRCurrPower = 0;
        double BLCurrPower = 0;

        if (Velocity) {
            // frontRight.setVelocity(FrontRight*max_velo);
            // frontLeft.setVelocity(FrontLeft*max_velo);
            // backRight.setVelocity(BackRight*max_velo);
            // backLeft.setVelocity(BackLeft*max_velo);
        } else {
            FLCurrPower = frontLeft.getPower();
            FRCurrPower = frontRight.getPower();
            BLCurrPower = backLeft.getPower();
            BRCurrPower = backRight.getPower();

            if (FrontRight == 0) {
                FRCurrPower = (-FRCurrPower) / 1;
            } else {
                frontRight.setPower(FrontRight);
            }
            if (FrontLeft == 0) {
                FLCurrPower = (-FLCurrPower) / 1;
            } else {
                frontLeft.setPower(FrontLeft);
            }
            if (BackRight == 0) {
                BRCurrPower = (-BRCurrPower) / 1;
            } else {
                backRight.setPower(BackRight);
            }
            if (BackLeft == 0) {
                BLCurrPower = (-BLCurrPower) / 1;
            } else {
                backLeft.setPower(BackLeft);
            }

            if (FrontRight == 0 )
            {
                frontRight.setPower(FRCurrPower);
            }
            if (FrontLeft == 0)
            {
                frontLeft.setPower(FLCurrPower);
            }
            if (BackRight == 0)
            {
                backRight.setPower(BRCurrPower);
            }
            if (BackLeft == 0)
            {
                backLeft.setPower(BLCurrPower);
            }


            if ((FrontLeft == 0) || (BackRight == 0) || (BackLeft == 0) || (FrontRight == 0)) {
                sleep(delay);
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);

            }


        }
    }
    private void rotatetoTargetHeading(int degrees, double power)
    {
        int error;
        double currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;



        // rotate until turn is completed.
        //Find where we are currently pointing
        currentHeading = getAngle();

        //Calculate how far off we are
        error = (int)(currentHeading - degrees);

        while(Math.abs(error) > 2){
            currentHeading = getAngle();

            //Calculate how far off we are
            error = (int)(currentHeading - degrees);

            //Using the error calculate some correction factor
            speedCorrection = Math.signum(error);

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speedCorrection * max_velo * power;
            rightSpeed = -speedCorrection * max_velo * power;

            backLeft.setVelocity(leftSpeed);
            frontLeft.setVelocity(leftSpeed);
            frontRight.setVelocity(rightSpeed);
            backRight.setVelocity(rightSpeed);

        }
        // turn the motors off.
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
    }
    private void rotatetoTargetHeadingWithZeroCount(int degrees, double power)
    {
        int error;
        double currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;
        int ZeroCount;



        // rotate until turn is completed.
        //Find where we are currently pointing
        currentHeading = getAngle();

        //Calculate how far off we are
        error = (int)(currentHeading - degrees);

        ZeroCount = 0;

        while((Math.abs(error) > 2) || (ZeroCount < 2)){
            currentHeading = getAngle();

            //Calculate how far off we are
            error = (int)(currentHeading - degrees);

            if (error == 0)
            {
                ZeroCount ++;
                ControlMotors(0,0,0,0,false,50);
                sleep(50);

            }
            else
            {
                ZeroCount = 0;
            }


            //Adjust the left and right power to try and compensate for the erro
            speedCorrection = CalculateCorrectionPower(error,0);
            leftSpeed = speedCorrection ;
            rightSpeed = -speedCorrection;

            ControlMotors(leftSpeed,rightSpeed,leftSpeed,rightSpeed,false,50);


        }
        // ensure motors are off
        ControlMotors(0,0,0,0,false,50);


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

    public void driveHeading(double distanceInInches, int targetHeading, double speed)
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
            speedCorrection = CalculateCorrectionPower(error,0);

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speed + speedCorrection;
            rightSpeed = speed - speedCorrection;

            leftSpeed= leftSpeed/2;
            rightSpeed= rightSpeed/2;

            //Apply the power settings to the motors
            ControlMotors(leftSpeed,rightSpeed,leftSpeed,rightSpeed,false,50);

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

        ControlMotors(0,0,0,0,false,100);

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