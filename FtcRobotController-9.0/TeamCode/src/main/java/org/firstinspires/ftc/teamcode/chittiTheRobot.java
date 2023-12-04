// File which contains the robot Configuration and functions.
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous_Mode.PipelineForAprilTagDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

public class chittiTheRobot
{
    // Access to methods in the opMode called.
    private final LinearOpMode myOpMode;
    private final Telemetry telemetry;

    // Motor Variables
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV    = 1440 ;    // Motor Encoder Counts
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // Circumference of the robot wheel
    public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public double driveSpeed = 0.6;
    public double turnSpeed = 0.5;

    //Distance Sensor Variable.
    DistanceSensor distanceSensor;

    // Servo Variables
    public Servo servo;

    // Camera Variables
    OpenCvCamera camera;
    PipelineForAprilTagDetection aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Calibrations values for C270 720p Camera equipped to the robot
    double fx = 822.317; double fy = 822.317; double cx = 319.495; double cy = 242.502;
    double tagSize = 0.166;

    //Declaring the Tag IDS we used in this program
    int leftTag = 1;
    int middleTag = 2;
    int rightTag = 3;
    public AprilTagDetection tagOfInterest = null;
    public boolean tagFound = false;

    // Constructor for chittiTheRobot class
    public chittiTheRobot(LinearOpMode opMode, Telemetry telemetry)
    {
        this.myOpMode = opMode;
        this.telemetry = telemetry;
    }

    // Initializing the hardware
    public void init()
    {
        leftWheel = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        servo = myOpMode.hardwareMap.get(Servo.class, "right_hand");

        distanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class,"sensor_distance");

        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap
                .get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new PipelineForAprilTagDetection(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
    }

    // This function checks the left, middle, or right tag AprilTag present in front of the robot.
    public void tagInformation()
    {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == leftTag || tag.id == middleTag || tag.id == rightTag)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
    }

    // Detected AprilTag Information.
    @SuppressLint("DefaultLocale")
    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

    // Robot Encoder Move Function
    public void Move(String string,double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftWheel.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightWheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftWheel.setTargetPosition(newLeftTarget);
            rightWheel.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftWheel.setPower(Math.abs(speed));
            rightWheel.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftWheel.isBusy() && rightWheel.isBusy()))
            {
                // Compare the string to use avoidCollision or not
                if(string.equals("GO"))
                {
                    telemetry.addLine("Going");
                }
                else if(string.equals("avoidCollision"))
                {
                    while (getDistance()<8){
                        telemetry.addData("Object present in Robot Path;","Waiting till it clears");
                        telemetry.update();
                        leftWheel.setPower(0);
                        rightWheel.setPower(0);
                    }
                    runtime.reset();
                    leftWheel.setPower(Math.abs(speed));
                    rightWheel.setPower(Math.abs(speed));
                }
            }

            // Stop all motion;
            leftWheel.setPower(0);
            rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250);
        }
    }
    //Function capable to get Distance
    public double getDistance()
    {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    // Robot Servo Rotating Function for Autonomous mode
    public void servoRun()
    {
        servo.setPosition(0);
        myOpMode.sleep(1500);
        servo.setPosition(1);
        myOpMode.sleep(1500);
    }

    // Turning the servo in direction specified for the Driver Mode.
    public void turnServo(String string)
    {
        if(string.equals("UP"))
        {
            servo.setPosition(0);
        }
        else if(string.equals("DOWN"))
        {
            servo.setPosition(1);
        }
    }
    // Function Responsible for going towards the Tag Detected
    public void goTowardsTag(int tag)
    {
        if(tag == leftTag)
        {
            // Going to the location where we drop the pixel
            Move("GO",driveSpeed,  -5.6,  -5.6, 5.0);
            Move("GO",turnSpeed,   -1.4, 1.4, 4.0);
            Move("GO",driveSpeed,-1.3,-1.3,3);
        }
        else if (tag== middleTag)
        {
            // Going to the location where we drop the pixel
            Move("GO",driveSpeed,  0.2,  -0.2, 5.0);
            Move("GO",turnSpeed,   -8.2, -8.2, 7.0);
        }
        else if (tag== rightTag)
        {
            // Going to the location where we drop the pixel
            Move("GO",driveSpeed,  -5.6,  -5.6, 5.0);
            Move("GO",turnSpeed,   1.4, -1.4, 4.0);
            Move("GO",driveSpeed,-1.3,-1.3,3);
        }
    }
    //Function Responsible to Park the robot
    public void Park(int position)
    {
        if(position == leftTag)
        {
            // Encoder values used for Parking left side
            Move("GO",driveSpeed,0.2,-0.2,3);
            Move("avoidCollision",driveSpeed, -19, -19, 15);
            Move("avoidCollision",driveSpeed, -1.4, 1.4, 3);
            Move("avoidCollision",driveSpeed, -3, -3, 4);
        }
        else if(position == middleTag)
        {
            // Encoder values used for Parking middle side
            Move("GO",driveSpeed,-4.2,-4.2,2);
            Move("GO",driveSpeed,-1.45,1.45,3);
            Move("avoidCollision",driveSpeed, -23, -23, 20);
            Move("GO",driveSpeed, -1.3, 1.3, 3);
            Move("avoidCollision",driveSpeed, -3, -3, 10);
        }
        else if (position == rightTag)
        {
            // Encoder values used for Parking right side
            Move("GO",driveSpeed,1.3,1.3,3);
            Move("GO",driveSpeed, -1.4, 1.4, 3);
            Move("avoidCollision",driveSpeed,-5.6,-5.6,10);
            Move("GO",driveSpeed, -1.4, 1.4, 3);
            Move("avoidCollision",driveSpeed,-21,-21,20);
        }
    }

}
