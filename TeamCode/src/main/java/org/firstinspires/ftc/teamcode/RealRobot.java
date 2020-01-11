
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="RealRobot", group = "Mechanicats")

// declaring the motors and servos our robot will use
public class RealRobot extends OpMode {
    // these are the four main wheels
    // that control the robot
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    // these are the intake wheels
    // at the front of the robot
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;
    private Gamepad g = null;
    // here is the servo that controls
    // the main arm
    private Servo outtake = null;
    // these are labeled "orange" and
    // "blue" for the mini intake arms
    private Servo orange = null;
    private Servo blue = null;
    private Servo Leftgrabber = null;
    private Servo Rightgrabber = null;


    public void init() {
        // this is so we know the names of all of our motors and servos
        // for configuration
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        // main wheels^^
        // below is the intake
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        // and here is the outtake servo
        outtake = hardwareMap.get(Servo.class, "outtake");
        // these are the orange and blue arms
        orange = hardwareMap.get(Servo.class, "orange");
        blue = hardwareMap.get(Servo.class, "blue");
        Leftgrabber = hardwareMap.get(Servo.class, "Leftgrabber");
        Rightgrabber = hardwareMap.get(Servo.class, "Rightgrabber");

        // sets the direction of all of the wheels.
        // the wheels on either side need need to be going
        // in the same direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // these are the intake wheels and they need to
        // be moving in opposite directions to get a hold on the stones
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    // this is where the code for
    // the controls start
    public void loop() {

        // setup of the controls for the new wheels
        double speed = .95;                       // controls the speed
        double vert = gamepad1.left_stick_y;      // vertical movement
        double strafe = gamepad1.left_stick_x;    // moving side to side
        double rotate = gamepad1.right_stick_x;   // controls orientation

        // this is where we control the position of the mini arms.
        // there is an open and close position for both arms
        // to reduce the amount of work done by the driver/s
        double blueClose = .5;
        double orangeClose = .5;
        double blueOpen = .5;
        double orangeOpen = .5;

        // here is the math tha ensures our robot moves correctly
        // with the mechanum wheels
        frontLeft.setPower((vert - strafe) * speed - rotate);
        backRight.setPower((vert - strafe) * speed + rotate);
        backLeft.setPower((vert + strafe) * speed - rotate);
        frontRight.setPower((vert + strafe) * speed + rotate);


        // intake on the front of the robot.
        // when the user presses the right or left
        // bumper on the controller, it starts the
        // intake wheels.
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
            telemetry.addLine("Bumpers/Intake");
        }
        // the intake is set to zero when not
        // being used
        else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        // controls for the outtake arm.
        // the arm will move up when the dpad is
        // pressed up
        if (gamepad1.dpad_up) {
            outtake.setPosition(0);
            telemetry.addLine("dpad UP");
        }

        // the arm will go down to to push the
        // block through the robot and onto
        // the foundation when the dpad is
        // pressed down
        if (gamepad1.dpad_down) {
            outtake.setPosition(1.00);
            telemetry.addLine("dpad DOWN");
        }

        // x and b buttons will control the mini arms.
        // the arms are labeled with orange and blue tape
        // so we know which one is which.

        // when x is pressed, the orange and blue arm closes
        if (gamepad1.x) {
            orange.setPosition(.8);
            blue.setPosition(.2);
        }
        // b opens the orange and blue arm
        if (gamepad1.b) {
            orange.setPosition(.2);
            blue.setPosition(.8);
        }

        telemetry.addData("left grabber: ", Leftgrabber.getPosition());
        telemetry.addData("Right grabber: ", Rightgrabber.getPosition());
        telemetry.update();

        if (gamepad1.left_trigger > .5){
            Leftgrabber.setPosition(.75);
            Rightgrabber.setPosition(.45);
        }
        else{
            Leftgrabber.setPosition(0);
            Rightgrabber.setPosition(0);
        }
    }

}

