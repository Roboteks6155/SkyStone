package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="IMUDriveSquare",group= "Example" )
public class IMUDriveSquare extends LinearOpMode {

    /* Declare OpMode member. */
    HardwarePushbot robot = new HardwarePushbot();

    Orientation angles;

     static final double GYRO_TOLERANCE = 5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        gyroDrive(30,.25,true);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        sleep(3000);

        robot.leftFrontDrive.setPower(0.5);
        robot.rightFrontDrive.setPower(0.5);
        robot.leftBackDrive.setPower(0.5);
        robot.rightBackDrive.setPower(0.5);
        sleep(2000);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        sleep(2000);

        gyroDrive(180,.25,false);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        sleep(3000);

        robot.leftFrontDrive.setPower(0.5);
        robot.rightFrontDrive.setPower(0.5);
        robot.leftBackDrive.setPower(0.5);
        robot.rightBackDrive.setPower(0.5);
        sleep(2000);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        sleep(2000);

        gyroDrive(90,.25,false);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        sleep(3000);

        robot.leftFrontDrive.setPower(0.5);
        robot.rightFrontDrive.setPower(0.5);
        robot.leftBackDrive.setPower(0.5);
        robot.rightBackDrive.setPower(0.5);
        sleep(2000);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        sleep(2000);

        gyroDrive(0,.25,false);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        sleep(3000);

        robot.leftFrontDrive.setPower(0.5);
        robot.rightFrontDrive.setPower(0.5);
        robot.leftBackDrive.setPower(0.5);
        robot.rightBackDrive.setPower(0.5);
        sleep(2000);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        sleep(2000);
    }

    public void gyroDrive(double targetAngle, double speed, boolean turnCCW){
        double currentHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(opModeIsActive()){
            while(opModeIsActive() && (Math.abs(targetAngle - currentHeading) >= GYRO_TOLERANCE)){
                currentHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                double sign = turnCCW ? -1 : 1;
                robot.leftBackDrive.setPower(speed * sign);
                robot.rightBackDrive.setPower(speed * -sign);
                robot.leftFrontDrive.setPower(speed * sign);
                robot.rightFrontDrive.setPower(speed * -sign);

                telemetry.addData("Base Power", speed);
                telemetry.addData("Angle", currentHeading);
                telemetry.addData("Error", (Math.abs(targetAngle - currentHeading)));
                telemetry.update();
            }
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
        }
    }
}