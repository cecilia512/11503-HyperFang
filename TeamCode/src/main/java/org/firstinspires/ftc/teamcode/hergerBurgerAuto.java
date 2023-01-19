package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


    @Autonomous(name="hergerBurger", group="auto")
    public class hergerBurgerAuto extends LinearOpMode {

        BNO055IMU imu;
        double globalAngle;
        Orientation lastAngles = new Orientation();

        hardwareMap mDrive = new hardwareMap();


        public final double WHEEL_DIAMETER = 4.0; //Wheel diameter in inches
        public final int MOTOR_GEAR_TEETH = 1; //# of teeth on the motor gear
        public final int WHEEL_GEAR_TEETH = 15; //# of teeth on the wheel gear
        public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
        public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //For every full turn of both motors, the wheel moves forward this many inches
        public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 28; //For every turn of the wheel


        int level;

        @Override
        public void runOpMode() throws InterruptedException {
          /*  mDrive.BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mDrive.FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mDrive.FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            mDrive.BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            mDrive.init(hardwareMap);
            imu.initialize(parameters);

            Vision vision = new Vision(this, 'r');

            telemetry.addData("MODE: ",mDrive.FR.getMode());
            telemetry.update();

            while (!isStarted() ) {
                level = vision.levelIdent('r');
                telemetry.addData("",level);
                telemetry.update();

            }

            waitForStart();
            if (!isStopRequested()){
                switch (level) {
                    case 1:
                        /*linearMovement(25.5,1.5, 0.0004, 0.00007, 0.000068);
                        sleep(595);
                        strafeMovement(1225, "LEFT");
                        sleep(1000);*/
                        turnDegree(90,5);
                        sleep(5000);
                        restBud();
                        break;
                    case 2:
                       /* linearMovement(30,1.5, 0.0004, 0.00007, 0.000068);
                        sleep(595);
                        sleep(1000);*/
                        turnDegree(90,5);
                        sleep(5000);
                        restBud();
                        restBud();
                        break;
                    default:
                        /*linearMovement(25.5,1.5, 0.0004, 0.00007, 0.000068);
                        sleep(595);
                        //larger on right idk why but it helps
                       strafeMovement(1225, "RIGHT");
                        sleep(1000);*/
                        turnDegree(90,5);
                        sleep(5000);
                        restBud();
                        restBud();
                        break;
                }
            }
        }


        public void restBud(){
            mDrive.resetEncoders();
            mDrive.BR.setVelocity(0);
            mDrive.BL.setVelocity(0);
            mDrive.FR.setVelocity(0);
            mDrive.FL.setVelocity(0);
        }
        public void strafeMovement(double rate, String direction){
            mDrive.resetEncoders();
            if (direction.equals("RIGHT")) {
                mDrive.FL.setVelocity(-rate); //right strafe
                mDrive.BL.setVelocity(rate);
                mDrive.FR.setVelocity(-rate);
                mDrive.BR.setVelocity(rate);
            }

            if (direction.equals("LEFT")) {
                mDrive.FL.setVelocity(rate); //left strafe
                mDrive.BL.setVelocity(-rate);
                mDrive.FR.setVelocity(rate);
                mDrive.BR.setVelocity(-rate);
            }
        }

        public void linearMovement(double distance, double tf, double kP, double kI, double kD) {
            mDrive.resetEncoders();
            double conversionIndex = 500.04; // ticks per inch
            double timeFrame = tf; //distance * distanceTimeIndex;
            double errorMargin = 5;
            double powerFloor = 0;
            double powerCeiling = 1;

            ElapsedTime clock = new ElapsedTime();
            clock.reset();
            mDrive.resetEncoders();

            double targetTick = -1 * distance * conversionIndex;
            telemetry.addData("target tick", targetTick);
            telemetry.update();


            double error = targetTick;
            double errorPrev = error;
            double time = clock.seconds();
            double timePrev = time;

            double  p, d, output;
            double i = 0;

            while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
                errorPrev = error;
                timePrev = time;

                double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();

                error = targetTick - tempAvg;
                time = clock.seconds();

                p = Math.abs(error) / 33.0 * kP;
                i += (time - timePrev) * Math.abs(error) / 33.0 * kI;
                d = Math.abs((error - errorPrev) / (time - timePrev) / 33.0 * kD);

                output = p + i - d;
                telemetry.addData("output", output);
                output = Math.max(output, powerFloor);
                output = Math.min(output, powerCeiling);
                if (error < 0) output *= -1;

                double currentAngle = imu.getAngularOrientation().firstAngle;
                double raw = globalAngle - currentAngle;
                if (raw > 180)
                    raw -= 360;
                if (raw < -180)
                    raw += 360;
                double fudgeFactor = 1.0 - raw / 40.0;

                if (distance > 0)
                {
                    mDrive.FL.setPower(output);
                    mDrive.FR.setPower(-output);
                    mDrive.BL.setPower(output);
                    mDrive.BR.setPower(-output); //-.35

                }
                else
                {
                    mDrive.FL.setPower(-output);
                    mDrive.FR.setPower(output);
                    mDrive.BL.setPower(-output);
                    mDrive.BR.setPower(output);

                }
            }
            mDrive.freeze();
        }
        /*public void strafeMovement(double distance,String direction , double tf, double kP, double kI, double kD) {
            mDrive.resetEncoders();
            double conversionIndex = 500.04; // ticks per inch
            double timeFrame = tf; //distance * distanceTimeIndex;
            double errorMargin = 5;
            double powerFloor = 0;
            double powerCeiling = 1;

            ElapsedTime clock = new ElapsedTime();
            clock.reset();
            mDrive.resetEncoders();

            double targetTick = -1 * distance * conversionIndex;
            telemetry.addData("target tick", targetTick);
            telemetry.update();


            double error = targetTick;
            double errorPrev = error;
            double time = clock.seconds();
            double timePrev = time;

            double  p, d, output;
            double i = 0;

            while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
                errorPrev = error;
                timePrev = time;

                double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();

                error = targetTick - tempAvg;
                time = clock.seconds();

                p = Math.abs(error) / 33.0 * kP;
                i += (time - timePrev) * Math.abs(error) / 33.0 * kI;
                d = Math.abs((error - errorPrev) / (time - timePrev) / 33.0 * kD);

                output = p + i - d;
                telemetry.addData("output", output);
                output = Math.max(output, powerFloor);
                output = Math.min(output, powerCeiling);
                if (error < 0) output *= -1;

                double currentAngle = imu.getAngularOrientation().firstAngle;
                double raw = globalAngle - currentAngle;
                if (raw > 180)
                    raw -= 360;
                if (raw < -180)
                    raw += 360;
                double fudgeFactor = 1.0 - raw / 40.0;

                if (distance > 0)
                {
                    mDrive.FL.setPower(output);
                    mDrive.FR.setPower(-output);
                    mDrive.BL.setPower(output);
                    mDrive.BR.setPower(-output); //-.35

                }
                else
                {
                    mDrive.FL.setPower(-output);
                    mDrive.FR.setPower(output);
                    mDrive.BL.setPower(-output);
                    mDrive.BR.setPower(output);

                }
            }
            mDrive.freeze();
       } */

        public void turnDegree(double degree, double timeframe) {

            mDrive.resetEncoders();

            telemetry.addLine("made it");
            telemetry.update();

            lastAngles = imu.getAngularOrientation();

            double currentAngle = lastAngles.firstAngle;
            ElapsedTime clock = new ElapsedTime();
            clock.reset();
            globalAngle += degree;
            if (globalAngle > 180)
                globalAngle -= 360;
            if (globalAngle < -180)
                globalAngle += 360;
            double leftPower, rightPower;

            // restart imu movement tracking.
            //resetAngle();
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            // set power to rotate.

            // rotate until turn is completed.

            double error = globalAngle - currentAngle;
            double errorPrev = error;

            double time = clock.seconds();
            double timePrev = time;

            double kPt = 0.045;//0.0118
            double kIt = 0.001;//0.005
            double kDt = 0.00;//0.002

            double p, d, output;
            double i = 0;

            while (clock.seconds() < timeframe && Math.abs(error) > 1 && opModeIsActive()) {
                lastAngles = imu.getAngularOrientation();
                currentAngle = lastAngles.firstAngle;

                timePrev = time;
                errorPrev = error;

                time = clock.seconds();
                error = globalAngle - currentAngle;



                if (error > 180)
                    error -= 360;
                if (error < -180)
                    error += 360;

                p = Math.abs(error) * kPt;
                i += (time - timePrev) * Math.abs(error) * kIt;
                d = ((Math.abs(error) - Math.abs(errorPrev)) / (time - timePrev)) * kDt;
                double conversionIndex = 500.04;
                output = (p + i + d) * conversionIndex;



                //telemetry.addData("output ", output);
                telemetry.addData("globalAngle", globalAngle);
                telemetry.addData("currentAngle", currentAngle);
                telemetry.addData("error ", error);
                //telemetry.addData("p", p);
                //telemetry.addData("i", i);
                //telemetry.addData("d", d);
                telemetry.update();


                if (error > 0)
                {
                    mDrive.FL.setVelocity(output);
                    mDrive.BL.setVelocity(output);
                    mDrive.FR.setVelocity(output);
                    mDrive.BR.setVelocity(output * 0.35);
                }
                else
                {
                    mDrive.FL.setVelocity(-output); //backwards
                    mDrive.BL.setVelocity(-output); //backwards
                    mDrive.FR.setVelocity(-output); //forwards
                    mDrive.BR.setVelocity(-output * 0.35); //forwards
                }
            }
            mDrive.freeze();
        }


    }
