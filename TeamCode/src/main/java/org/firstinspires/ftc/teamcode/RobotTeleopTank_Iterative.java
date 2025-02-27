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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This particular OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")
//@Disabled
public class RobotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  leftFront;
    public DcMotor  rightFront;
    public DcMotor  rightBack;
    public DcMotor  leftBack;
    public DcMotorEx  liftOne;
    public DcMotorEx liftTwo;
    public CRServo  vexClaw;

    double clawOffset = 0;

    /*public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        //vexArmL     = hardwareMap.crservo.get( "vexArmL");
        //vexArmR     = hardwareMap.crservo.get("vexArmR");
        liftOne     = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftOne");
        liftTwo     = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftTwo");
        vexClaw     = hardwareMap.crservo.get("vexClaw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
 //       leftClaw  = hardwareMap.get(Servo.class, "left_hand");
 //       rightClaw = hardwareMap.get(Servo.class, "right_hand");
 //       leftClaw.setPosition(MID_SERVO);
  //      rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double horizontal;
        double vert;
        double turn;
        double tog = 1.00;
        //double tog2 = 1.00;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)

       /* if ( gamepad1.a){

        }*/
        if(gamepad1.right_bumper) {tog = .6;} //maybe switch to speed - trigger
        if(gamepad1.left_bumper) {tog = 1.00;}
        if(gamepad2.right_bumper) {tog = .75;} //maybe switch to speed - trigger
        if(gamepad2.left_bumper) {tog = 1.00;}

        horizontal =  0.7 * gamepad1.left_stick_x;
        vert  = -.7 * gamepad1.left_stick_y;
        turn  = 0.7 * gamepad1.right_stick_x;

        double lb = vert + turn - horizontal;
        double lf = vert + turn + horizontal;
        double rb = vert - turn + horizontal; //right strafe having an issue
        double rf = vert - turn - horizontal;

        double lbco = lb; //- 0.12 + (.35 / ( lb + 0.6));
        double lfco = lf; //- 0.12 + (.35 / ( lf + 0.6));
        double rbco = rb; //- 0.12 + (.35 / ( rb + 0.6));
        double rfco = rf; // - 0.12 + (.35 / ( rf + 0.6));

        leftBack.setPower(tog*lbco*.88);
        leftFront.setPower(tog*lfco*.88);
        rightBack.setPower(tog*rbco*1.15*.88);
        rightFront.setPower(tog*rfco*.88);

        double clawOpen    = gamepad2.right_trigger * .6;// * TRY .5 oscilloscope
        double clawClose  = -gamepad2.left_trigger * .5;// * 1.5; //.25 IF YOU SEE JERKIN GLOWER POWER, CANNOT GO OVER .85

        telemetry.addData("sirvo right",  "%.2f", clawOpen);
        telemetry.addData("sirvo left ",  "%.2f", clawClose);

        if ( clawClose == 0 ) vexClaw.setPower(clawOpen);
        if ( clawOpen == 0 )   vexClaw.setPower(clawClose);

        /*double armPower = 0;
        if (gamepad2.right_bumper){
            armPower = -1.0;
            vexArmL.setPower(armPower);
            vexArmR.setPower(armPower);
        }
        else if (gamepad2.left_bumper){
            armPower = 1.0;
            vexArmL.setPower(armPower);
            vexArmR.setPower(armPower);
        }
        else{
            armPower = 0;
            vexArmL.setPower(armPower);
            vexArmR.setPower(armPower);
        }*/
        /*double armPower = -gamepad2.right_stick_y;

        if ( armPower != 0 && !gamepad2.b){
            if (gamepad2.b) armPower = 0;
            vexArmL.setPower( armPower * .8);
            vexArmR.setPower(-armPower * .8);
            telemetry.addData("armpower ",  "%.2f", armPower);ba
        }*/


        double liftPower = 0;
        double yVal = gamepad2.right_stick_y;
        telemetry.addData("Liftpower: ", "%.2f", liftPower);
        telemetry.addData("y val : ", gamepad2.right_stick_y);

        if (gamepad2.right_stick_y > .3 || gamepad2.right_stick_y < -.3){

            if ( gamepad2.b ) liftPower = 0;
            else if (yVal < -.3 || yVal > .3){
                liftPower = -gamepad2.right_stick_y * 8000; // * tog2;
            }

            if (!gamepad2.b) {
                liftOne.setVelocity(-liftPower);
                liftTwo.setVelocity(liftPower);
            }

        }
        else {
            liftPower = 0;
            liftOne.setVelocity(-liftPower);
            liftTwo.setVelocity(liftPower);
        }


        /*if (gamepad2.right_bumper || gamepad2.left_bumper){
            double liftPower = 0;
            if (gamepad2.dpad_down){
                liftPower = -.6;
            }
            else if (gamepad2.dpad_up){
                liftPower = .6;
            }
            else{
                liftPower = 0;
            }
            //vexArmL.setPower(liftPower);
            //vexArmR.setPower(liftPower);
            tempArm.setPower(liftPower);
            telemetry.addData("armpower ",  "%.2f", liftPower);
        }*/





        /* Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED; //CHANGE TO VARIABLE !!!! <3

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        leftClaw.setPosition(MID_SERVO + clawOffset);
        rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            leftArm.setPower(ARM_UP_POWER);
        else if (gamepad1.a)
            leftArm.setPower(ARM_DOWN_POWER);
        else
            leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);*/
        telemetry.addData("horizontal",  "%.2f", horizontal);
        telemetry.addData("vertical", "%.2f", vert);
        telemetry.addData("turn", "%.2f", turn);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
