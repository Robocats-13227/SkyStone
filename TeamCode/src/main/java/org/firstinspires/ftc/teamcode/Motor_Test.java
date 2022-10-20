package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "motor_test" ,group = "Mechanicats")
public class Motor_Test extends LinearOpMode {
    @Override
    public void runOpMode()  {

        DcMotorEx Test_motor = hardwareMap.get(DcMotorEx.class ,"Arm");
        Test_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        double arm_power  = gamepad1.right_trigger - gamepad1.left_trigger;
        if(isStopRequested()) return;
        while (opModeIsActive()){
            if (gamepad1.left_trigger >0 || gamepad1.right_trigger >0)
                Test_motor.setVelocity(1000 * arm_power);

            telemetry.addData("test",Test_motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
