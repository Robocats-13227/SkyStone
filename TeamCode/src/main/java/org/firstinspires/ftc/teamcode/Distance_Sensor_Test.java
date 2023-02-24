package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name = "Distance_Sense_test",group = "Mechanicats")
public class Distance_Sensor_Test extends LinearOpMode {
    Rev2mDistanceSensor Pole_Senor = null;
    double globalAngle= 0;
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        Pole_Senor = hardwareMap.get(Rev2mDistanceSensor.class,"Pole_Sensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive())
        {
            Distance_read();
        }

    }

    private double getAngle ()
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

  /*  public void Distance_Rotate(Boolean left , double power)
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
        if(left)
        {
            while(Pole_Senor.getDistance(DistanceUnit.INCH) >10){




                //Using the error calculate some correction factor
                speedCorrection = Math.signum(1);

                //Adjust the left and right power to try and compensate for the error
                leftSpeed = speedCorrection  * power;
                rightSpeed = -speedCorrection  * power;

                backLeft.setPower(leftSpeed);
                frontLeft.setPower(leftSpeed);
                frontRight.setPower(rightSpeed);
                backRight.setPower(rightSpeed);

            }
        }
        else
        {

        }
    }
   */
    public void Distance_read()
    {
        while (opModeIsActive())
        {
            Pole_Senor.getDistance(DistanceUnit.INCH);
            telemetry.addData("pole Sensor",Pole_Senor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
