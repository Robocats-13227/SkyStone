package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="OpModeTest2", group = "Linear Opmode")

public class OpModeTest2 extends OpMode {

    private DcMotor motorFr = null;
    private DcMotor motorFl = null;
    private DcMotor motorBr = null;
    private DcMotor motorBl = null;



    public void init(){
        motorFr = hardwareMap.get(DcMotor.class, "motorFr");
        motorFl = hardwareMap.get(DcMotor.class, "motorFl");
        motorBr = hardwareMap.get(DcMotor.class, "motorBr");
        motorBl = hardwareMap.get(DcMotor.class, "motorBl");

        motorFl.setDirection(DcMotor.Direction.REVERSE);
        motorFr.setDirection(DcMotor.Direction.FORWARD);
        motorBl.setDirection(DcMotor.Direction.REVERSE);
        motorBr.setDirection(DcMotor.Direction.FORWARD);
    }
    public void loop(){
        motorFr.setPower(gamepad1.right_stick_y);
        motorFl.setPower(gamepad1.left_stick_y);
        motorBr.setPower(gamepad1.right_stick_y);
        motorBl.setPower(gamepad1.left_stick_y);

        if (gamepad1.a) {

            motorFr.setPower(.5);
            motorFl.setPower(.5);
            motorBr.setPower(.5);
            motorBl.setPower(.5);

        }

        if (gamepad1.b) {

            motorFr.setPower(-.5);
            motorFl.setPower(-.5);
            motorBr.setPower(-.5);
            motorBl.setPower(-.5);

        }




        }


    }
