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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;




/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumPreSeason", group="Preseason")
//@Disabled
public class MecanumTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSkystone robot           = new HardwareSkystone();   // Use a Pushbot's hardware



   // public Servo servoRepositioning = null;
    // double          clawOffset      = 0;                       // Servo mid position
   // final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //variable that assigns what joystick will give the power for vertical movement
            double cascadingHorizontalPower = gamepad2.left_stick_x;
            double cascadingVerticalPower = gamepad2.left_stick_y;

            //variable that assigns what trigger will give power to each compliant wheel
            double rightCollectorPower = gamepad1.right_trigger;
            double leftCollectorPower = gamepad1.left_trigger;

            // makes it so that translation is on the left joystick and rotation/turning is on the right joystick
            double forward = -gamepad1.left_stick_y/2;
            double strafe = gamepad1.left_stick_x/2;
            double rotate = gamepad1.right_stick_x/2;



            //sets the power of the motor controlling the vertical movement
            robot.cascadingVerticalArm.setPower(cascadingVerticalPower);
            robot.cascadingHorizontalArm.setPower(cascadingHorizontalPower);

            //sets the power of the motors controlling the compliant wheels
            robot.leftCollectorMotor.setPower(leftCollectorPower);
            robot.rightCollectorMotor.setPower(-rightCollectorPower);

            //method that is used to make the robot move
            MecanumMove(forward, strafe, rotate);

            //an IF loop to spin the compliant wheels outwards
            if (gamepad1.b) {
                robot.leftCollectorMotor.setPower(-leftCollectorPower);
                robot.rightCollectorMotor.setPower(rightCollectorPower);
            }

            //an IF loop to rotate the repositioning arm up and down
        if (gamepad1.y){
            robot.servoRepositioning.setPosition(0.32);
        } else if (gamepad1.a){
            robot.servoRepositioning.setPosition(0);
        }

           //an IF loop for the cascading claw
           if (gamepad2.dpad_left){
                robot.armServo.setPosition(0);
           } else if (gamepad2.dpad_right){
                robot.armServo.setPosition(0.388888888888);
            }

           if (gamepad2.y){
                robot.pickerArmServo.setPosition(0);
            }
            else if (gamepad2.a){
                robot.pickerArmServo.setPosition(0.32);
            }
            if (gamepad2.left_bumper) {
                robot.pickerClawServo.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                robot.pickerClawServo.setPosition(0.32);
            }
        }

    }

    // Easier to understand and make changes when considering turning and strafing right
    //This method is made to move the robot in all directions using the only two joysticks
    private void MecanumMove(double forward, double strafe, double rotate) {

        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        // Normalize the values so neither exceed +/- 1.0
       double max = Math.max(Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)), Math.max(Math.abs(rightFrontPower), Math.abs(leftFrontPower)));

        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //Sets the powers of the wheels
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
    }

}
