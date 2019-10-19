package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSkystone;
import org.firstinspires.ftc.teamcode.utils.SkystoneFollower;
import org.firstinspires.ftc.teamcode.utils.VuforiaSkystoneDetector;

@Autonomous(name="TrackSkystone",group= "Skystone" )

public class TrackSkystone extends LinearOpMode {
   /* Declare OpMode members. */
    HardwareSkystone robot           = new HardwareSkystone();   // Use a Pushbot's hardware
    VuforiaSkystoneDetector vuforia = new VuforiaSkystoneDetector(true);
    SkystoneFollower follower = new SkystoneFollower();

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

        while (opModeIsActive()) {
            vuforia.gatherData();

            if(vuforia.isTargetVisible()) {
                double xOffset = vuforia.getXTranslation();
                double yOffset = vuforia.getYTranslation();

                telemetry.addData("X Offset", xOffset);
                telemetry.addData("Y Offset", yOffset);

                double[] outputs = follower.calculateOutput(xOffset, yOffset);
                double strafe = outputs[0], forward = outputs[1];
                follower.setYTarget(-1.0);
                if(!follower.withinTolerance()) {
                    MecanumMove(-forward, -strafe, 0);
                } else {
                    MecanumMove(0, 0, 0);
                }
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Forward", forward);
                telemetry.update();
            } else {
                telemetry.addData("Target:", "Not Visible");
                telemetry.update();
                MecanumMove(0, 0, 0);
            }

        }
        vuforia.stop();
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
        robot.leftBackDrive.setPower(-leftBackPower);
        robot.rightBackDrive.setPower(-rightBackPower);
        robot.leftFrontDrive.setPower(-leftFrontPower);
        robot.rightFrontDrive.setPower(-rightFrontPower);
    }
}
