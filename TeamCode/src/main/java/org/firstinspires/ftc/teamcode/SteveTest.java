/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SteveTest", group="Linear Opmode")

//@Disabled
public class SteveTest extends LinearOpMode {

    final private boolean dashboardEnabled = false;//Are we debugging with FtcDashboard ?
    final private int gyroOrientation = -1;//Set to either +1 or -1 depending on control hub mountint orientation
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx ArmMotor = null;
    private Servo Claw_servo = null;
    private Servo Claw2_servo = null;
    private BNO055IMU imu    = null;
    private double translateJoyX = 0.0;
    private double translateJoyY = 0.0;
    private double translateTargetJoyX = 0.0;
    private double translateTargetJoyY = 0.0;
    private double translateDirection;
    private double translateSpeed;
    private double targetHeading;
    private double currentHeading;
    private double botCentricDirection = 0.0;
    private double headingError;
    private double normalizationAngle;
    private double translationSpeed = 0.0;
    private double normalizationFactor = 1.0;
    private double headingCorrectionSpeed = 0.0;
    private double frontLeftSpeed;
    private double frontRightSpeed;
    private double backLeftSpeed;
    private double backRightSpeed;
    private double rotateClockwiseTrigger;
    private double rotateAnticlockwiseTrigger;
    private boolean rotateNorth = false;
    private boolean rotateSouth = false;
    private boolean rotateEast  = false;
    private boolean rotateWest  = false;

    private double Lj_init_x;
    private double Lj_init_y;

    private double Rj_init_x;
    private double Rj_init_y;

    //Tunable variables
    final private double translationUpdateFactor = 0.05;//Translation joystick tracking rate
    final private double headingCorrectionFactor = 360.0 / 10000.0;//Heading rotation correction 'power' factor
    final private double minHeadingError = 2.0;//Minimim heading error to actuall act on
    final private double headingCorrectionSpeedMax = 0.5;//The maximum rotation power factor
    final private int    max_base_velocity = 2000;//Max robot base translation velocity
    final private int    max_arm_velocity= 2000;
    final private int    DriveMode = 1;
    final private double translateDeadband = 0.05;//Minimum speed factor. Will need to tune
    final private int    BOTTOM_ARM_POS = 400;
    final private int    LOW_ARM_POS =  1925; // = 13 inches
    final private int    MEDIUM_ARM_POS =3147  ; // = 23 inches
    final private int    TOP_ARM_POS = 4344; // = 33 inches
    final private  int   ArmMoveSpeed = 200;//Ticks per processing loop to move in manual mode
    final private double TRIGGER_DEADZONE = 0.1;

    /**
     * Initialize the motors
     */
    private void initializeMotors(){
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BackRight");
        ArmMotor = hardwareMap.get(DcMotorEx.class,"Arm");
        Claw_servo = hardwareMap.get(Servo.class,"Claw");
        Claw2_servo = hardwareMap.get(Servo.class,"Claw2");


        // Motors on one side need to effectively run 'backwards' to move 'forward'
        // Reverse the motors that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(max_arm_velocity);

        resetEncoders();
    }

    public void initializeJoysticks(){
        //Capture the 'at rest' positions of the joysticks
        Lj_init_x = gamepad1.left_stick_x;
        Lj_init_y = gamepad1.left_stick_y;

        Rj_init_x = gamepad1.right_stick_x;
        Rj_init_y = gamepad1.right_stick_y;
    }

    private void resetEncoders(){
        //Stop the motors and reset the encoders to zero
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //Make sure we re-enable the use of encoders
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize the gyro
     */
    private void initializeGyro(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private double getGyroHeading(){
        //  return gyro.getIntegratedZValue();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void waitStart(){
        // Wait for the game to start (driver presses PLAY)
        //Can use this time to process the vision in order to get the vision marker ASAP
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
    }

    /**
     * Disable all hardware
     */
    private void disableHardware() {
        //Only the motors really need to be turned off at the moment
        setMotors(0, 0, 0, 0);
    }

    /**
     * Initialize all hardware
     */
    private void initializeHardware(){
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();

        initializeMotors();
        initializeGyro();
        initializeJoysticks();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized hardware");
        telemetry.update();
    }

    /**
     * Get the current robot gyro heading in degrees
     */
    public double getCurrentHeading(){
        double current;

        current = getGyroHeading() * gyroOrientation;
        //Convert to +- 180 in radians
        while (current > 180)  current -= 360;
        while (current <= -180) current += 360;
        return current;
    }

    /**
     * Calculate the heading error and limit to +- 180 degrees. Result in degrees
     * @param targetHeading, currentHeading
     */

    public double calculateHeadingError(double targetHeading, double currentHeading) {
        double robotError;

        // calculate error in -179 to +180 range : In degrees
        robotError = targetHeading - currentHeading;
        while (robotError >= 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

        //If error is small then ignore it
        if   ((robotError > -minHeadingError) && (robotError < minHeadingError))
        {
            robotError = 0.0;
        }
        return -robotError;
    }

    private void setMotors(double FL, double FR, double BL, double BR){
        double max;
        //If any value is greater than 1.0 normalize all values accordingly
        max = Math.max(Math.max(Math.max(FL,FR),Math.max(BL,BR)), 1.0);
        FL = FL / max;
        FR = FR / max;
        BL = BL / max;
        BR = BR / max;
        //Depending on whether using power mode or velocity mode...
        if (DriveMode == 0)
        {
            frontLeftDrive.setPower(FL);
            frontRightDrive.setPower(FR);
            backLeftDrive.setPower(BL);
            backRightDrive.setPower(BR);
        }
        else
        {
            frontLeftDrive.setVelocity(FL * max_base_velocity);
            frontRightDrive.setVelocity(FR * max_base_velocity);
            backLeftDrive.setVelocity(BL * max_base_velocity);
            backRightDrive.setVelocity(BR * max_base_velocity);
        }
    }

    private void updateJoysticks()
    {
        translateTargetJoyX =    -gamepad1.left_stick_x ;
        translateTargetJoyY =    gamepad1.left_stick_y ;

        //For the moment don't do any acceleration control
        translateJoyX = translateTargetJoyX;
        translateJoyY = translateTargetJoyY;

        /*
        if (translateJoyX + translationUpdateFactor > translateTargetJoyX)
        {
            translateJoyX = translateTargetJoyX;
        }
        else
        {
            translateJoyX = translateJoyX + translationUpdateFactor;
        }
        //Joystick Y tracking
        if (translateTargetJoyY > translateJoyY)
        {
            if (translateJoyY + translationUpdateFactor > translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY + translationUpdateFactor;
            }
        }
        else if (translateTargetJoyY < translateJoyY)
        {
            if (translateJoyY - translationUpdateFactor < translateTargetJoyY)
            {
                translateJoyY = translateTargetJoyY;
            }
            else
            {
                translateJoyY = translateJoyY - translationUpdateFactor;
            }
        }
         */

        rotateAnticlockwiseTrigger =   gamepad1.left_trigger;
        rotateClockwiseTrigger =  gamepad1.right_trigger;
        rotateNorth = gamepad1.dpad_up;
        rotateSouth = gamepad1.dpad_down;
        rotateEast  = gamepad1.dpad_right;
        rotateWest  = gamepad1.dpad_left;
    }

    void updateTargetTranslation()
    {
        //Calculate desired translation 'speed' and  direction
        translateSpeed = Math.sqrt((translateJoyX * translateJoyX) + (translateJoyY * translateJoyY));

        if (translateSpeed > translateDeadband){//Create deadband and also ensure no divide by zero in atan2 and stop robot twitching
            //Calculate the desired robot base direction
            //Forward = 0 radians (0 degrees)
            translateDirection = Math.toDegrees(-Math.atan2(translateJoyY, translateJoyX) + (Math.PI/2));
        }
        else {
            translateDirection = 0;
            translateSpeed = 0;
        }
    }

    void updateTargetHeading()
    {
        if (rotateNorth)
        {
            targetHeading = 0;
        }
        else if (rotateSouth)
        {
            targetHeading = 180;
        }
        else if (rotateWest)
        {
            targetHeading = 90;
        }
        else if (rotateEast)
        {
            targetHeading = -90;
        }

    }

    void calculateMotorSpeeds()
    {
        //Note, the diagonally opposite speeds are the same, so only need to calculate 2 values
        frontLeftSpeed = translationSpeed*Math.sin(Math.toRadians(botCentricDirection)+(Math.PI/4));

        frontRightSpeed = translationSpeed*Math.cos(Math.toRadians(botCentricDirection+(Math.PI/4)));

        headingCorrectionSpeed = headingError * headingCorrectionFactor;

        //Clamp the rotational component to 0.5
        if (headingCorrectionSpeed > headingCorrectionSpeedMax)
        {
            headingCorrectionSpeed = headingCorrectionSpeedMax;
        }
        else if (headingCorrectionSpeed < -headingCorrectionSpeedMax)
        {
            headingCorrectionSpeed = -headingCorrectionSpeedMax;
        }


        //Duplicate diagonal speeds for translation and add in rotation
        backLeftSpeed   = frontRightSpeed + headingCorrectionSpeed;
        backRightSpeed  = frontLeftSpeed - headingCorrectionSpeed;
        frontRightSpeed = frontRightSpeed - headingCorrectionSpeed;
        frontLeftSpeed  = frontLeftSpeed + headingCorrectionSpeed;
    }

    private void doTeleop()
    {
        //Take our initial zero heading from our starting pose
        targetHeading = getCurrentHeading();

        while(opModeIsActive())
        {
            currentHeading = getCurrentHeading();
            //Get the joystick states
            updateJoysticks();
            //Calculate translation speed and direction from joysticks
            updateTargetTranslation();
            //Update rotation heading
            //           updateTargetHeading();
            //Calculate any heading error
            headingError = calculateHeadingError(targetHeading, currentHeading);
            botCentricDirection = translateDirection;
            //Now calculate the actual motor speeds
            calculateMotorSpeeds();
            //And actually set the motors accordingly
            //Note, this function will also clamp and scale the power to 1.0
            setMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);


//            process_lifter();

//            process_claw();


            telemetry.addData("FL: ", frontLeftSpeed);
            telemetry.addData("FR: ", frontRightSpeed);
            telemetry.addData("BL: ", backLeftSpeed);
            telemetry.addData("BR: ", backRightSpeed);
            telemetry.addData("ArmMotor",ArmMotor.getCurrentPosition());
            telemetry.addData("heading",currentHeading);
            telemetry.addData("targetHeading",targetHeading);
            telemetry.addData("headingCorrectionSpeed",headingCorrectionSpeed);
            telemetry.addData("heading error degrees",headingError);

            telemetry.update();
        }
        setMotors(0, 0, 0, 0);
    }

    /**
     * This is the main op mode and should call all the initialization, wait for start,
     * execute your desired auto/tele-op, then stop everything
     */
    @Override
    public void runOpMode() {

        //Must be called at the start to initialize all necessary hardware
        //Add other hardware (e.g. vision etc...) in this method
        initializeHardware();

        if (dashboardEnabled) {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        }
        // Wait for the game to start (driver presses PLAY)
        //Note, we can use this time to be processing the vision to have the skystone location ready ASAP.
        waitStart();
        runtime.reset();

        doTeleop();

        //Done so turn everything off now
        disableHardware();
    }

}