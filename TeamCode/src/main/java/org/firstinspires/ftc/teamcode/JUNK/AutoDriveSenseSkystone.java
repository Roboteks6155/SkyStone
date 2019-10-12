package org.firstinspires.ftc.teamcode.JUNK;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareSkystone;
import org.firstinspires.ftc.teamcode.PreSeasonTests.HardwarePushbot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//This code was not functioning correctly
@Disabled
@Autonomous(name="AutoDriveSenseSkystone",group= "Example" )
public class AutoDriveSenseSkystone extends LinearOpMode {

    /* Declare OpMode member. */
    HardwareSkystone robot = new HardwareSkystone();

    Orientation angles;

    //mandatory variables needed to use a phone for Vuforia
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    public final double skystoneTolerance = 1.00 * mmPerInch;

    // Vuforia key the whole team can use
    private static final String VUFORIA_KEY =
            "AUQjOCH/////AAABmZze+n8bwEqgsTx67M3N4s9FOi2kjnhS5MO9kw2EMXMGPEzG/j4XKth779vKOyVLgv4G0Bs78WSls3O6DnJPAR+aZD5pce07IukhOBq2JVK0Vuv/uGVw+xb1vcgzD2cXcXv2askajp6vWAHmTf0/TeiRcn128n00LvkGP4sKMMVsDtCxdKYThMGxCYVWI8bSIsJi6ikraV9jkqjuV83+FzHQFGKSttT/9kr3nMZc7tuXC3lL2vgkFt0Q+azowmajL5PuCCgNblSsTemP8y2dOeQ9QJTkf+wImOXS/2OJAiD7IDaCc0jiF1M2p7EnBEUCnpeypjrVjA05rGF+AaXgsVvH+x9BTO77yfxWdIFXFuqi";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    //Variable to store skystone position
    public String skystonePosition = "Pos 1";
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    //WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    static final double GYRO_TOLERANCE = 5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Code will run over the hardware file
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //Reset encoders to make sure the encoders start at 0
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set mode for the motors to run using an encoder
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition(),
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        //Wait for the Drivers to click button
        waitForStart();
        encoderDrive(0.5,10,10,3);
       sleep(7000);

        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;

            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData("Visible Target Test1", trackable.getName());
                telemetry.addData("IsVisible Test2", ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible());
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());

                    if( trackable.getName().equals("Stone Target")) {
                        telemetry.addLine("Skystone is visible");
                    }
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // These are variables to find how far you are from the center of the Skystone on the x and y axis
                double skystoneXOffset = translation.get(0) / mmPerInch;
                double skystoneYOffset = translation.get(1) / mmPerInch;

                //Variable to store the position of the Skystone if on the right
                if (skystoneXOffset > CAMERA_LEFT_DISPLACEMENT * mmPerInch){
                    skystonePosition = "Pos 1";
                    telemetry.addData("Skystone Position", skystonePosition);
                }

                //Variable to store the position of the Skystone if on the left
                if (skystoneXOffset < CAMERA_LEFT_DISPLACEMENT * mmPerInch){
                    skystonePosition = "Pos 2";
                    telemetry.addData("Skystone Position", skystonePosition);
                }

                telemetry.addData("Skystone Position", skystonePosition);

                //if the Skystone is at postion 1
                while ( CAMERA_LEFT_DISPLACEMENT * mmPerInch - skystoneXOffset >= skystoneTolerance ){
                    robot.leftBackDrive.setPower(0.4);
                    robot.leftFrontDrive.setPower(-0.4);
                    robot.rightBackDrive.setPower(0.4);
                    robot.rightFrontDrive.setPower(-0.4);
                }

                //if the Skystone is at postion 2
                while ( CAMERA_LEFT_DISPLACEMENT * mmPerInch - skystoneXOffset <= skystoneTolerance ){
                    robot.leftBackDrive.setPower(-0.4);
                    robot.leftFrontDrive.setPower(0.4);
                    robot.rightBackDrive.setPower(0.4);
                    robot.rightFrontDrive.setPower(-0.4);
                }


                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();

       //SenseSkystone();

        telemetry.addData("Skystone Position", skystonePosition);

        encoderDrive(0.4,20,20,3);
        //put picking up skystone mechanism coder here


        encoderDrive(0.3,7,7,3);


        //make robot strafe to under the line
        if (skystonePosition == "Pos 1"){
            robot.leftBackDrive.setPower(-0.8);
            robot.leftFrontDrive.setPower(0.8);
            robot.rightFrontDrive.setPower(-0.8);
            robot.rightBackDrive.setPower(0.8);
            sleep(2500);
        }

        if (skystonePosition == "Pos 2"){
            robot.leftBackDrive.setPower(0.8);
            robot.leftFrontDrive.setPower(-0.8);
            robot.rightFrontDrive.setPower(0.8);
            robot.rightBackDrive.setPower(-0.8);
            sleep(3000);
        }

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH_GOBILDA);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH_GOBILDA);
            newLeftFrontTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH_GOBILDA);
            newRightFrontTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH_GOBILDA);

            //Tell each motor what it's target/distance is
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
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

    /*
    Method to use the in built IMU/(Inertial Measurement Unit)/Gyro to turn more accurately
    */
    public void gyroDrive(double targetAngle, double speed, boolean turnCCW){

        //variable that gets what angle you REV is at
        double currentHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //to check if the OpMode is still active
        if(opModeIsActive()){
            while(opModeIsActive() && (Math.abs(targetAngle - currentHeading) >= GYRO_TOLERANCE)){
                currentHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                //Setting either the power -1 or 1 to the wheels, depending on which way it needs to turn
                double sign = turnCCW ? -1 : 1;
                robot.leftBackDrive.setPower(speed * sign);
                robot.rightBackDrive.setPower(speed * -sign);
                robot.leftFrontDrive.setPower(speed * sign);
                robot.rightFrontDrive.setPower(speed * -sign);

                //Telemetry for gyroDrive
                telemetry.addData("Base Power", speed);
                telemetry.addData("Angle", currentHeading);
                telemetry.addData("Error", (Math.abs(targetAngle - currentHeading)));
                telemetry.update();
            }

            //Setting the wheels powers to 0 so it stops after reach its target angle
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);


        }
    }

    public void SenseSkystone() {

    }
}
