// This file is used for testing the robot movements based on the encoder counts

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.chittiTheRobot;

@TeleOp(name="Encoder_Testing", group="Testing")

public class Encoder_Testing extends LinearOpMode
{
    chittiTheRobot miniBot4;

    @Override
    public void runOpMode()
    {
        miniBot4 = new chittiTheRobot(this,telemetry);
        miniBot4.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Values to Test the Path(distance) and Speed
        double driveSpeed = 0.6;
        double turnSpeed = 0.5;
        miniBot4.Move("GO", driveSpeed,  0.29,  -0.29, 5.0);
        miniBot4.Move("avoidCollision", turnSpeed,   -4.7, -4.7, 4.0);

    }

}
