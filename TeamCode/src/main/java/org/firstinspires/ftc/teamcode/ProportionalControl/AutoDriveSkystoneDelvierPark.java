package org.firstinspires.ftc.teamcode.ProportionalControl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSkystone;
import org.firstinspires.ftc.teamcode.ProportionalControl.SkystoneFollower;
import org.firstinspires.ftc.teamcode.ProportionalControl.SkystonePosition;
import org.firstinspires.ftc.teamcode.ProportionalControl.VuforiaSkystoneDetector;

@Autonomous(name = "AutoDriveSkystoneDelvierPark", group = "Skystone")

public class AutoDriveSkystoneDelvierPark extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSkystone robot = new HardwareSkystone();   // Use a Pushbot's hardware
    VuforiaSkystoneDetector vuforia = new VuforiaSkystoneDetector(true);
    SkystoneFollower follower = new SkystoneFollower();
    SkystonePosition skystonePosition = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        vuforia.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        vuforia.start();

        if (opModeIsActive()) {
            encoderDrive(0.2, 14, 14, 3);
            sleep(750);                                         //See Roshans program - runtime is reset here
            while (opModeIsActive()) {                                    //See Roshans program - while condition is different
                vuforia.gatherData();
                                                                          //See Roshans program - while loop is closed here
                if (skystonePosition == null) {
                    if (vuforia.getXTranslation() < 0) {
                        telemetry.addData("X Offset", vuforia.getXTranslation());
                        telemetry.update();
                        sleep(3000);

                        skystonePosition = SkystonePosition.RIGHT;
                    } else if (vuforia.getXTranslation() > 0) {
                        skystonePosition = SkystonePosition.CENTER;
                    } else {
                        skystonePosition = SkystonePosition.LEFT;
                        encoderStrafe(0.2, 5, 3, false);
                    }
                    telemetry.addData("Position", skystonePosition.toString());
                    telemetry.update();
                    trackTarget();
                }


            }
        }
        vuforia.stop();
    }

    private void trackTarget() {
        while (vuforia.isTargetVisible() && !follower.withinTolerance()) {          //See Roshans program - different condition
                                                                                    //See Roshans program - gatherData here
            double xOffset = vuforia.getXTranslation();
            double yOffset = vuforia.getYTranslation();

            telemetry.addData("X Offset", xOffset);
            telemetry.addData("Y Offset", yOffset);

            double[] outputs = follower.calculateOutput(xOffset, yOffset);
            double strafe = outputs[1], forward = outputs[0];                       //See Roshans program - interchanged
            follower.setYTarget(-1.0);

            MecanumMove(forward/2, strafe/2, 0);                  //See Roshans program - not halved

            telemetry.addData("Is target visable", vuforia.isTargetVisible());
            telemetry.addData("Is within tolerance", follower.withinTolerance());
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward", forward);
            telemetry.update();
        }
        MecanumMove(0, 0, 0);
//         else {
//            telemetry.addData("Target:", "Not Visible");
//            telemetry.update();
//            MecanumMove(0, 0, 0);
//        }
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

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //Sets the powers of the wheels
        robot.leftBackDrive.setPower(-leftBackPower);                                   //See Roshans program - not negative values
        robot.rightBackDrive.setPower(-rightBackPower);                                 //See Roshans program - not negative values
        robot.leftFrontDrive.setPower(-leftFrontPower);                                 //See Roshans program - not negative values
        robot.rightFrontDrive.setPower(-rightFrontPower);                               //See Roshans program - not negative values
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        //variables that are used to set the target distance the wheel is going
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //Reset encoders to make sure the encoders start at 0
            robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH_GOBILDA);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH_GOBILDA);
            //newLeftFrontTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH_GOBILDA);
            //newRightFrontTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH_GOBILDA);


            //Tell each motor what it's target/distance is
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightBackTarget);
            //robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            //robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
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

            // optional pause after each move
            sleep(250);
        }
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
}
