// This file is used for testing servo arm position before starting the Autonomous mode

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.chittiTheRobot;

@TeleOp(name = "Servo_Testing", group = "Testing")

public class Servo_Testing extends LinearOpMode {
    chittiTheRobot miniBot4;
    @Override
    public void runOpMode() {
        miniBot4 = new chittiTheRobot(this,telemetry);
        miniBot4.init();
        waitForStart();

        while(opModeIsActive()){
            // Adjust the arm position of the servo by running this function
            miniBot4.servoRun();
        }
    }
}
