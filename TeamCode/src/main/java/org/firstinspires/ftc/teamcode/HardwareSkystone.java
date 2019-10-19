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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareSkystone
{
    /* Public OpMode members. */

    //Robot motor variables
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor cascadingVerticalArm = null;
    public DcMotor cascadingHorizontalArm = null;
    public DcMotor rightCollectorMotor = null;
    public DcMotor leftCollectorMotor = null;
    //Encoder Variables
    public final double     DRIVE_SPEED             = 0.6;
    public final double     TURN_SPEED              = 0.5;
    static public final double     COUNTS_PER_MOTOR_REV_GOBILDA    = 753.2 ;    // For Gobilda Motor Encoder
    static public final double     DRIVE_GEAR_REDUCTION    = 0.6666 ;     // This is < 1.0 if geared UP
    public final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final double     COUNTS_PER_INCH_GOBILDA         = (COUNTS_PER_MOTOR_REV_GOBILDA * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double INCHES_PER_ROTATION_STRAFING = 10.6;
    static public final double STRAFING_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_GOBILDA * DRIVE_GEAR_REDUCTION) / (INCHES_PER_ROTATION_STRAFING);
    //Proportional Control Variables
    public double TOLERANCE = 5;
    public double PROPORTIONAL_CONSTANT;
    public double leftBackError = 10;
    public double rightBackError = 10;
    public double currentLeftBackInches;
    public double currentRightBackInches;
    public double rightOutput;
    public double leftOutput;

    // Robot Servo variables
    public Servo servoRepositioning;
    public Servo servoCascadingClaw;
    public Servo pickerArmServo;
    public Servo pickerClawServo;
    public Servo leftCollectorServo;
    public Servo rightCollectorServo;
    public boolean isReposition = false;
    public boolean isCascadingClaw = false;
    public boolean ispickerClaw = false;
    public boolean ispickerArm = false;
    public boolean isIntake = false;
    public final double SERVO0 = 0.32;
    public final double SERVO90 = 0;


    //Variables for using IMU/Gyro
    public BNO055IMU imu = null;
    static final public double GYRO_TOLERANCE = 2.5;

    // Robot Sensor Variables
    ColorSensor colorSensor;

    public WebcamName webcamName = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSkystone(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hwMap.get(DcMotor.class,"leftBackDrive");
        rightBackDrive = hwMap.get(DcMotor.class,"rightBackDrive");
        leftCollectorMotor = hwMap.get(DcMotor.class, "leftCollectorMotor");
        rightCollectorMotor = hwMap.get(DcMotor.class, "rightCollectorMotor");
        cascadingVerticalArm = hwMap.get(DcMotor.class, "cascadingVerticalArm");
        cascadingHorizontalArm = hwMap.get(DcMotor.class, "cascadingHorizontalArm");

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hwMap.get(WebcamName.class, "Webcam");

        servoCascadingClaw = hwMap.get(Servo.class,"CascadingClawServo");
        servoRepositioning = hwMap.get(Servo.class,"servoRepositioning");
        pickerArmServo = hwMap.get(Servo.class,"pickerArmServo");
        pickerClawServo =  hwMap.get(Servo.class,"pickerClawServo");
        leftCollectorServo = hwMap.get(Servo.class, "leftCollectorServo");
        rightCollectorServo = hwMap.get(Servo.class, "rightCollectorServo");
        colorSensor =  hwMap.get(ColorSensor.class, "colorsensor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightCollectorMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        servoRepositioning.setDirection(Servo.Direction.REVERSE);// Set direction of the Servo

        // initialize positions of servos
        servoRepositioning.setPosition(SERVO0);
        pickerArmServo.setPosition(SERVO0);



        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // leftArm.setPower(0);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
}