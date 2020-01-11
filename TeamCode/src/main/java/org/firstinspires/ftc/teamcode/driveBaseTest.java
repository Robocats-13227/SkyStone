package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "driveBaseTest",group = "Mechanicats")

public class driveBaseTest extends LinearOpMode {


    private DcMotor Right = null;
    private DcMotor Left = null;


    private double pi= 3.141;
    private double ticksPerRev = 1100;
    private double wheelDiameter = 9;
    private double distanceFactor =(pi * wheelDiameter) / ticksPerRev;
    //private int driveBaseMotors = 4;


    @Override
    public void runOpMode() throws InterruptedException {

        Right = hardwareMap.get(DcMotor.class, "frontRight");
        Left = hardwareMap.get(DcMotor.class, "frontLeft");

        Right.setDirection(DcMotor.Direction.REVERSE);
        Left.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        waitForStart();

        // WHERE THINGS ACTUALLY START TO HAPPEN
        forward(140,.6);
        //grab
        backwards(130, .6);
        //let go
        forward(20, .6);
        turnRight();
        forward(34, .6);
    }

    //makes robot go forward using parameters distance and power while checking whether values are true
    private void forward(double distance, double power){

        //gets current position from current of left and right motors, averages it out and sets overall current position
        int currentPosition = (Left.getCurrentPosition() + Right.getCurrentPosition()) / 2;
        // gets distance divided by distance factor to set wanted position
        double wantedPosition = distance / distanceFactor;

        //when current position is lower than wanted motors will spin
        while(currentPosition < wantedPosition){
            Right.setPower(power);
            Left.setPower(power);
        }

        //once current position no longer lower than wanted motors stop
        Right.setPower(0);
        Left.setPower(0);
    }
    //turns right
    private void turnRight(){
        Right.setPower(.6);
        Left.setPower(0);
    }
    //turns left
    private void turnLeft(){
        Right.setPower(0);
        Left.setPower(.6);
    }
    //resets motors to 0
    private void resetEncoders(){
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    private void backwards(double distance, double power) {
        int currentPosition = (Left.getCurrentPosition() + Right.getCurrentPosition()) / 2;
        double wantedPosition = distance / distanceFactor;
        Right.setDirection(DcMotor.Direction.FORWARD);
        Left.setDirection(DcMotor.Direction.REVERSE);
    }

}
