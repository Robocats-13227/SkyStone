package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "motor_test" ,group = "Mechanicats")
public class Motor_Test extends LinearOpMode {

    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx Arm_Motor = null;
    private Servo Claw_servo = null;
    private Servo Claw_2 =null;

    public void initialize()
    {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        Claw_servo = hardwareMap.get(Servo.class,"Claw");
        Claw_2 = hardwareMap.get(Servo.class,"Claw2");

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





    }
    public void runOpMode()  {


        initialize();

        waitForStart();
        while (opModeIsActive()) {




            motorFrontRight.setPower(gamepad1.left_trigger*10);
            telemetry.addData("motor ticks",motorFrontRight.getCurrentPosition());
            updateTelemetry(telemetry);
        }




        }
    }

