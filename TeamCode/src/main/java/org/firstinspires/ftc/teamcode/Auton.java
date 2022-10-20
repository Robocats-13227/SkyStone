package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@Autonomous(name = "auto" ,group = "Mechanicats")
public class Auton extends LinearOpMode {

    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx Arm_Motor = null;


    public void runOpMode() {
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

        }
    }
}
