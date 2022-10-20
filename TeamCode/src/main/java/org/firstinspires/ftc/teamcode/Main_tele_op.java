package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name="MAIN TELE_OP", group = "Mechanicats")

public class Main_tele_op extends LinearOpMode {
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx Arm_Motor = null;
    private Servo Claw_servo = null;

    double joystick_1x = 0;
    double joystick_1y = 0;

    double target_1x = 0;
    double target_1y = 0;

    final public double Stick_interval = .2;

    BNO055IMU imu;

    public int servo_pos;

    final public int BOTTOM_ARM_POS = 0;
    //TODO get encoder value
    final public int LOW_ARM_POS =  4727; // = 13 inches

    //TODO get encoder value
    final public int MEDIUM_ARM_POS = 8363 ; // = 23 inches

    final public int TOP_ARM_POS = 4000; // = 33 inches


    final public double TRIGGER_DEADZONE = 0.1;

    public int max_velo= 2000;

    public int ArmMoveSpeed = 200;

    public void process_lifter()
    {

        int current_pos = Arm_Motor.getCurrentPosition();
        if (( Arm_Motor.getCurrentPosition() < TOP_ARM_POS-ArmMoveSpeed) && (gamepad2.right_trigger > TRIGGER_DEADZONE) )
        {
                Arm_Motor.setTargetPosition(current_pos+ArmMoveSpeed);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_Motor.setVelocity(max_velo);
        }
        else if((Arm_Motor.getCurrentPosition() > BOTTOM_ARM_POS+ArmMoveSpeed) &&  (gamepad2.left_trigger > TRIGGER_DEADZONE))
        {
            Arm_Motor.setTargetPosition(current_pos-ArmMoveSpeed);
            Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm_Motor.setVelocity(max_velo);
        }

            else if (gamepad2.dpad_down)
            {
                Arm_Motor.setTargetPosition(BOTTOM_ARM_POS);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_Motor.setVelocity(max_velo);
            }
            else if (gamepad2.dpad_right)
            {
                Arm_Motor.setTargetPosition(LOW_ARM_POS);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_Motor.setVelocity(max_velo);

            }
            else if (gamepad2.dpad_left)
            {
                Arm_Motor.setTargetPosition(MEDIUM_ARM_POS);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_Motor.setVelocity(max_velo);

            }
            else if (gamepad2.dpad_up) {
                Arm_Motor.setTargetPosition(TOP_ARM_POS);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_Motor.setVelocity(max_velo);
            }




    }

    public void initialize()
    {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        Claw_servo = hardwareMap.get(Servo.class,"Claw");
        Claw_servo.setPosition(0);

        Arm_Motor = hardwareMap.get(DcMotorEx.class,"Arm");


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm_Motor.setTargetPosition(BOTTOM_ARM_POS);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_Motor.setVelocity(max_velo);


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public void process_motion()
    {
        double y = -joystick_1y; // Remember, this is reversed!
        double x = -joystick_1x; // Counteract imperfect strafing
        double rx;
        double target_angle= 0;
        double current_angle;
        double error_angle;


        current_angle = imu.getAngularOrientation().thirdAngle;
        error_angle= current_angle-target_angle;
        rx = error_angle/150;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setVelocity(frontLeftPower* max_velo);
        motorBackLeft.setVelocity(backLeftPower * max_velo);
        motorFrontRight.setVelocity(frontRightPower * max_velo);
        motorBackRight.setVelocity(backRightPower * max_velo);
    }
    public void process_joystick()
    {

        target_1x = gamepad1.left_stick_x;
        target_1y = gamepad1.left_stick_y;

        if(target_1x > joystick_1x )
        {
            if(joystick_1x +Stick_interval > target_1x)
            {
                joystick_1x = target_1x;
            }
            else
            {
                joystick_1x+=Stick_interval;
            }
        }
        if (target_1x < joystick_1x)
        {
            if(joystick_1x -Stick_interval < target_1x)
            {
                joystick_1x = target_1x;
            }
            else
            {
                joystick_1x-=Stick_interval;
            }
        }


         if(target_1y > joystick_1y )
         {
             if(joystick_1y +Stick_interval > target_1y)
             {
                 joystick_1y = target_1y;
             }
             else
             {
             joystick_1y+=Stick_interval;
             }
         }
        if (target_1y < joystick_1y)
        {
            if(joystick_1y -Stick_interval < target_1y)
            {
                joystick_1y = target_1y;
            }
            else
            {
                joystick_1y-=Stick_interval;
            }
        }

        telemetry.addData("joystick_valuex",target_1x);
        telemetry.addData("joystick_valuey",target_1y);
        telemetry.addData("joystick_Targetvaluex",joystick_1x);
        telemetry.addData("joystick_Targetvaluey",joystick_1y);
        telemetry.update();
    }

    public void process_claw()
    {
        if (gamepad2.x)
            Claw_servo.setPosition(1);
        else if (gamepad2.b)
            Claw_servo.setPosition(0);
    }


    public void runOpMode() {

        initialize();

        waitForStart();

        //if (isStopRequested()) return;
        while (opModeIsActive()) {

            process_joystick();

            process_motion();

            process_lifter();

            process_claw();


            telemetry.addData("arm encoder",Arm_Motor.getCurrentPosition());

            //telemetry.addData("claw encoder",Claw_servo.getPosition());

            telemetry.addData("claw encoder", 0);
            updateTelemetry(telemetry);
        }
    }
}



