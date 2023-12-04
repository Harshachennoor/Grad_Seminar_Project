// Autonomous File used at GamePlay.
package org.firstinspires.ftc.teamcode.Autonomous_Mode;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.chittiTheRobot;


@Autonomous(name="Autonomous_GO", group="Game")
public class Autonomous_GO extends LinearOpMode
{
    chittiTheRobot miniBot4;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        miniBot4 = new chittiTheRobot(this,telemetry);
        miniBot4.init();

        //The INIT-loop:This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested())
        {
            miniBot4.tagInformation();
            if(miniBot4.tagFound)
            {
                // Update the telemetry after April Tag found
                telemetry.addLine(String.format("\nDetected tag ID=%d", miniBot4.tagOfInterest.id));
                miniBot4.tagToTelemetry(miniBot4.tagOfInterest);
            }
            telemetry.update();
            sleep(20);
        }

        // The START command just came in
        // Action taken based on the April Tag detected.
        telemetry.addData("Going towards","Tag ID: %d",miniBot4.tagOfInterest.id);
        telemetry.update();

        //go Towards the tag
        miniBot4.goTowardsTag(miniBot4.tagOfInterest.id);
        sleep(500);

        // Drop the Pixel and update the telemetry
        telemetry.addData("Dropping:","Purple Pixel");
        miniBot4.servoRun();
        sleep(500);

        // Park the robot and update the Telemetry
        telemetry.addData("Going towards","Parking Area");
        telemetry.update();
        miniBot4.Park(miniBot4.tagOfInterest.id);

    }
}
