// File used in Driver Mode at GamePLay
package org.firstinspires.ftc.teamcode.Driver_Mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.chittiTheRobot;

@TeleOp(name="Driver Mode", group="Game")
public class Driver_GO extends LinearOpMode
{
    double leftWheelPower;
    double rightWheelPower;
    chittiTheRobot miniBot4;

    @Override
    public void runOpMode()
    {
        miniBot4 = new chittiTheRobot(this,telemetry);
        miniBot4.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            //Turning the servo arm based on the button pressed.
            if (gamepad1.y)
            {
                miniBot4.turnServo("UP");
            }
            if(gamepad1.a)
            {
                miniBot4.turnServo("DOWN");
            }
            // Moving the Robot with GamePad1 controls.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftWheelPower = Range.clip(drive + turn, -1.0, 1.0) ;
            rightWheelPower = Range.clip(drive - turn, -1.0, 1.0) ;
            miniBot4.leftWheel.setPower(leftWheelPower);
            miniBot4.rightWheel.setPower(rightWheelPower);
        }
    }
}
