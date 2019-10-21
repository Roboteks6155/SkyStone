package org.firstinspires.ftc.teamcode.PreSeasonTests.FrogForceTraining;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PreSeasonTests.HardwarePushbot;

@Disabled
@Autonomous(name="ProportionalControl",group= "Example" )
public class ProportionalControl extends LinearOpMode {

    /* Declare OpMode member. */
    HardwarePushbot robot = new HardwarePushbot();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("left Encoder Counts", getLeftEncoderCounts());
            telemetry.addData("right Encoder Counts", getRightEncoderCounts());
            telemetry.addData("left Encoder Counts", getLeftEncoderCounts() / robot.COUNTS_PER_INCH);
            telemetry.addData("right Encoder Counts", getRightEncoderCounts() / robot.COUNTS_PER_INCH);
            telemetry.update();

       driveToPosition(20);
        }

    }

    public double getLeftEncoderCounts() {
        return robot.leftBackDrive.getCurrentPosition();
    }

    public double getRightEncoderCounts() {
        return robot.rightBackDrive.getCurrentPosition();

    }

    final double TOLERANCE = 5;
    final double PROPORTIONAL_CONSTANT = 0.1;
    double leftError;
    double rightError;

    public void driveToPosition(double targetDistance) {
        while (opModeIsActive() && !((Math.abs(leftError) < TOLERANCE) && (Math.abs(rightError) < TOLERANCE))) {

            double currentLeftInches = getLeftEncoderCounts() / robot.COUNTS_PER_INCH;
            double currentRightInches = getRightEncoderCounts() / robot.COUNTS_PER_INCH;

            double leftError = targetDistance - currentLeftInches;
            double rightError = targetDistance - currentRightInches;

            double leftOutput = PROPORTIONAL_CONSTANT * leftError;
            double rightOutput = PROPORTIONAL_CONSTANT * rightError;

            robot.leftBackDrive.setPower(leftOutput);
            robot.rightBackDrive.setPower(rightOutput);

            sleep(100);
        }
    }
}
