/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.PreSeasonTests.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSkystone;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="MecanumMethod", group="Pushbot")
@Disabled
 public class MecanumMethod extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSkystone robot = new HardwareSkystone();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 753.2;    // eg: Gobilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.66666666;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double INCHES_PER_ROTATION = (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double INCHES_PER_ROTATION_STRAFING = 10.6;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (INCHES_PER_ROTATION);
    static final double STRAFING_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (INCHES_PER_ROTATION_STRAFING);


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Strafing counts per inch", STRAFING_COUNTS_PER_INCH);
        sleep(3000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Location", "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // DRIVE_SPEED = 0.6
        // TURN_SPEED = 0.5

        encoderStrafe(true, 24, 0.5, 10);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderStrafe(boolean directionRight, double inches, double strafingSpeed, int timeoutS) {
        //  left is false, right is true
        int NewLeftFrontTarget;
        int NewRightFrontTarget;
        int NewLeftBackTarget;
        int NewRightBackTarget;

        if (opModeIsActive()) {

            if (!directionRight) {
                NewLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (-inches * STRAFING_COUNTS_PER_INCH);
                NewRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (-inches * STRAFING_COUNTS_PER_INCH);
                NewLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (inches * STRAFING_COUNTS_PER_INCH);
                NewRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (inches * STRAFING_COUNTS_PER_INCH);
            } else {
                NewLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (inches * STRAFING_COUNTS_PER_INCH);
                NewRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (inches * STRAFING_COUNTS_PER_INCH);
                NewLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (-inches * STRAFING_COUNTS_PER_INCH);
                NewRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (-inches * STRAFING_COUNTS_PER_INCH);
            }
            telemetry.addData("current Postion",robot.leftBackDrive.getCurrentPosition());
            robot.leftFrontDrive.setTargetPosition(NewLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(NewRightFrontTarget);
            robot.leftBackDrive.setTargetPosition(NewLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(NewRightBackTarget);

            telemetry.addData("NewLeftBackTarget", NewLeftBackTarget);
            telemetry.update();
            sleep(2000);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFrontDrive.setPower(strafingSpeed);
            robot.rightFrontDrive.setPower(strafingSpeed);
            robot.leftBackDrive.setPower(strafingSpeed);
            robot.rightBackDrive.setPower(strafingSpeed);

            // This while loop is NECESSARY to keep the motor running to its position
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", NewLeftBackTarget,  NewRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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

            // optional pause after each move
            sleep(250);
        }
    }
}
