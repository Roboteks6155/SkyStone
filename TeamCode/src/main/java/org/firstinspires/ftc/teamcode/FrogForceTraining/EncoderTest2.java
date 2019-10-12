package org.firstinspires.ftc.teamcode.FrogForceTraining;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PreSeasonTests.HardwarePushbot;

@Autonomous(name="EncoderTest2",group= "Example" )
public class EncoderTest2 extends LinearOpMode {

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

        while (opModeIsActive()){
            telemetry.addData("left Encoder Counts", getLeftEncoderCounts());
            telemetry.addData("right Encoder Counts", getRightEncoderCounts());
            telemetry.addData("left Encoder Counts", getLeftEncoderCounts()/robot.COUNTS_PER_INCH);
            telemetry.addData("right Encoder Counts", getRightEncoderCounts()/robot.COUNTS_PER_INCH);
        }

    }

    public double getLeftEncoderCounts() {
        return robot.leftBackDrive.getCurrentPosition();
        }
    public double getRightEncoderCounts() {
    return robot.rightBackDrive.getCurrentPosition();

    }

}

