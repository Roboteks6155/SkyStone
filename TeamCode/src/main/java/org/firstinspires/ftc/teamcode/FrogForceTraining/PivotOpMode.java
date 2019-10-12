package org.firstinspires.ftc.teamcode.FrogForceTraining;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PreSeasonTests.HardwarePushbot;
@Disabled
@Autonomous(name="PivotOpMode",group= "Example" )
public class PivotOpMode extends LinearOpMode {

    /* Declare OpMode member. */
    HardwarePushbot robot = new HardwarePushbot();

    Orientation angles;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            final double targetAngle = 10;
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentAngle = angles.firstAngle;
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();

            pivotToAngle(targetAngle, currentAngle);
        pivotToAngle(10, currentAngle);

        }
    }

    private void pivotToAngle(double TargetHeading, Double currentHeading) {
        final double TOLERANCE = 3;
        final double BASE_POWER = 0.5;
        if (Math.abs(TargetHeading - currentHeading) >= TOLERANCE) {
            double leftPower = BASE_POWER;
            double rightPower = -BASE_POWER;

                robot.leftBackDrive.setPower(leftPower);
                robot.rightBackDrive.setPower(rightPower);
            }
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }
    }
