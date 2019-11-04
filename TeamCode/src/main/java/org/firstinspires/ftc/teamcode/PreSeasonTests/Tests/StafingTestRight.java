package org.firstinspires.ftc.teamcode.PreSeasonTests.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareSkystone;

//@Disabled
@Autonomous(name="StrafingTestRight",group= "Test" )
public class StafingTestRight extends LinearOpMode {

    /* Declare OpMode member. */
    HardwareSkystone robot = new HardwareSkystone();

    Orientation angles;

     static final double GYRO_TOLERANCE = 3.0;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        encoderStrafe(0.3,60,10,true);
    }

    public void encoderStrafe(double strafingSpeed, double inches, int timeoutS, boolean directionRight) {
        //  left is false, right is true
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {

            //Reset encoders to make sure the encoders start at 0
            robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if (!directionRight) {
                newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (-inches * robot.STRAFING_COUNTS_PER_INCH);
                newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (-inches * robot.STRAFING_COUNTS_PER_INCH);
                newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (inches * robot.STRAFING_COUNTS_PER_INCH);
                newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (inches * robot.STRAFING_COUNTS_PER_INCH);
            } else {
                newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (inches * robot.STRAFING_COUNTS_PER_INCH);
                newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (inches * robot.STRAFING_COUNTS_PER_INCH);
                newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (-inches * robot.STRAFING_COUNTS_PER_INCH);
                newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (-inches * robot.STRAFING_COUNTS_PER_INCH);
            }
            telemetry.addData("current Postion", robot.leftBackDrive.getCurrentPosition());

            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            telemetry.addData("NewLeftBackTarget", newLeftBackTarget);
            telemetry.update();
            sleep(100);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.leftFrontDrive.setPower(strafingSpeed);
            robot.rightFrontDrive.setPower(strafingSpeed);
            robot.leftBackDrive.setPower(strafingSpeed);
            robot.rightBackDrive.setPower(strafingSpeed);

            // This while loop is NECESSARY to keep the motor running to its position
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
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
                telemetry.update();
            }
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
        }
    }
}