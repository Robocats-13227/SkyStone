package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="Mech OpMode", group = "Mechanicats")

public class opModeChallenge extends OpMode {

    private DcMotor motorRight = null;
    private DcMotor motorLeft = null;
    private DcMotor inLeft = null;
    private DcMotor inRight = null;

    public void init() {
        motorRight = hardwareMap.get(DcMotor.class, "Right Motor");
        motorLeft = hardwareMap.get(DcMotor.class, "Left Motor");
        inRight = hardwareMap.get(DcMotor.class, "Right In");
        inLeft = hardwareMap.get(DcMotor.class, "Left In");


        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {

        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight.setPower(gamepad1.right_stick_y);

        // spinny doodads
        // if (gamepad1.a) {
        //   inLeft.setPower(1);
        // inRight.setPower(1);
        //}
        //if (gamepad1.a)
        //  motorLeft.setPower(1);
        //motorRight.setPower(1);
    } //
}