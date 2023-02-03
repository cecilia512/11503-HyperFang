/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class legomanrightside extends LinearOpMode
{

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 11, 5, 3 from the 36h11 family (11503 yoyoyoyoyo)
    int left = 11;
    int middle = 5;
    int right = 3;
    AprilTagDetection tagOfInterest = null;

    goofyahhapriltagreturning gahr = new goofyahhapriltagreturning();

    @Override
    public void runOpMode()
    {
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        gahr.init(this, aprilTagDetectionPipeline);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /*gahr.linearMovement(29,1.5, 0.0004, 0.00007, 0.000068);
         sleep(595);*/
        gahr.clawGetr(0.5);
        sleep(3000);
        gahr.linearMovement(36,1.5, 0.0004, 0.00007, 0.000068);
        sleep(595);
        stop();
        gahr.strafeMovement(588, "LEFT");
        sleep(1100);
        stop();
        gahr.strafeMovement(0,"LEFT");
        sleep(10);
        gahr.badLiftMovement(4000);
        sleep(4000);
        gahr.badLiftMovement(0);
        sleep(10);
        gahr.linearMovement(6.75, 1.5, 0.0004, 0.00007, 0.000068);
        sleep(1000);
        stop();
        gahr.mover(0);
        sleep(100);
        stop();
            /*gahr.mover(-200);
            sleep(600);
            gahr.mover(0);
            sleep(10);
            stop();*/
        gahr.badLiftMovement(-1500);
        sleep(2000);
        stop();
        gahr.badLiftMovement(0);
        sleep(10);
        gahr.clawGetr(-0.5);
        sleep( 700);
        gahr.clawGetr(0);
        sleep(10);
        stop();
        gahr.badLiftMovement(1000);
        sleep(1500);
        gahr.badLiftMovement(0);
        sleep(0);
        gahr.mover(500);
        sleep(500);
        gahr.mover(0);
        sleep(10);
        gahr.clawGetr(0.5);
        sleep(700);
        gahr.clawGetr(0);
        stop();
        gahr.badLiftMovement(-2000);
        sleep(3000);




        /*gahr.linearMovement(-5, 1.5, .0004, .00007, .000068);
        sleep(1000);*/
        gahr.restBud();
        if (tagOfInterest == null || tagOfInterest.id == left) {
            //very old turn parking stuff?
            /*gahr.linearMovement(36,1.5, 0.0004, 0.00007, 0.000068);
            sleep(595);
            gahr.turnDegree(-3,.15);
            sleep(150);
            gahr.strafeMovement(600, "LEFT");
            //gahr.turnDegree(70, .5);
            sleep(1000);
            //gahr.lift(4000);

            //sleep(1000);
            //gahr.linearMovement(31,1.5, .0004, .00007, .000068);
            gahr.restBud();*/
            //
            gahr.linearMovement(1, 1.5, 0.0004, 0.00007, 0.000068);
            sleep(300);
            gahr.strafeMovement(700, "LEFT");
            sleep(950);

        } else if (tagOfInterest.id == middle) {
            //strafe
            gahr.linearMovement(1, 1.5, 0.0004, 0.00007, 0.000068);
            sleep(300);
            gahr.strafeMovement(700, "RIGHT");
            sleep(750);

        } else {
            //old code
            /*gahr.linearMovement(36,1.5, 0.0004, 0.00007, 0.000068);
            sleep(595);
            //gahr.turnDegree(6, .1);
            //sleep(100);
            gahr.strafeMovement(1150, "RIGHT");

            sleep(1000);
            //gahr.linearMovement(31,1.5, .0004, .00007, .000068);
            gahr.restBud();*/

            //very janky case 3 strafe
            gahr.linearMovement(1.5, 1.5, 0.0004, 0.00007, 0.000068);
            sleep(200);
            gahr.strafeMovement(1000, "RIGHT");
            sleep(1000);
            gahr.linearMovement(3, 1.5, 0.0004, 0.00007, 0.000068);
            sleep(200);
            gahr.strafeMovement(800, "RIGHT");
            sleep(650);

        }
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}