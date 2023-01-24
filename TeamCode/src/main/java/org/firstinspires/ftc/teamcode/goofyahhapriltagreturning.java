package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardwareMap;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class goofyahhapriltagreturning {
    public final double WHEEL_DIAMETER = 4.0; //Wheel diameter in inches
    public final int MOTOR_GEAR_TEETH = 1; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 15; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 28; //For every turn of the wheel

    hardwareMap mDrive = new hardwareMap();
    double globalAngle;
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;
    OpenCvCamera camera;

    LinearOpMode opMode;
    Telemetry telemetry;

    public void init(LinearOpMode opMode, OpenCvPipeline my_pipeline) {
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();

        param.mode = BNO055IMU.SensorMode.IMU;
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled = false;

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        mDrive.init(hardwareMap);
        imu.initialize(param);

        if (my_pipeline != null) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            camera.setPipeline(my_pipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }
        telemetry = opMode.telemetry;
        telemetry.setMsTransmissionInterval(50);
    }

    public void lift(double distance){
        telemetry.addLine("made it");
        telemetry.update();
        mDrive.resetEncoders();
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        double timeframe = 1;
        double error = distance;
        double conversionIndex = 536;
        double targetTick = distance * conversionIndex;
        double errorPrev = error;

        double time = clock.seconds();
        double timePrev = time;

        double kP = 0.0118;
        double kI = 0.005;
        double kD = 0.002;
        double p, d, output;
        double i = 0;
        while (clock.seconds() < timeframe && Math.abs(error) > 1 && opMode.opModeIsActive()){
            errorPrev = error;
            timePrev = time;

            double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();
            error = targetTick - tempAvg;
            time = clock.seconds();

            p = Math.abs(error) * kP;
            i += (time - timePrev) * Math.abs(error) * kI;
            d = ((Math.abs(error) - Math.abs(errorPrev)) / (time - timePrev)) * kD;

            output = p + i - d;

            if ( distance < 0 ){
                mDrive.liftOne.setVelocity(-output);
                mDrive.liftOne.setVelocity(output);
            }
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

        double p, d, output;
        double i = 0;

        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opMode.opModeIsActive()) {
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

            if (distance > 0) {
                mDrive.FL.setPower(output);
                mDrive.FR.setPower(-output);
                mDrive.BL.setPower(output);
                mDrive.BR.setPower(-output); //-.35

            } else {
                mDrive.FL.setPower(-output);
                mDrive.FR.setPower(output);
                mDrive.BL.setPower(-output);
                mDrive.BR.setPower(output);

            }
        }
        mDrive.freeze();
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

    public void turnDegree(double degree, double timeframe) {
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

        double kP = 0.0118;
        double kI = 0.005;
        double kD = 0.002;

        double p, d, output;
        double i = 0;

        while (clock.seconds() < timeframe && Math.abs(error) > 1 && opMode.opModeIsActive()) {
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

            p = Math.abs(error) * kP;
            i += (time - timePrev) * Math.abs(error) * kI;
            d = ((Math.abs(error) - Math.abs(errorPrev)) / (time - timePrev)) * kD;

            output = p + i + d;



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
                mDrive.FL.setPower(output);
                mDrive.BL.setPower(output);
                mDrive.FR.setPower(0);
                mDrive.BR.setPower(output);
            }
            else
            {
                mDrive.FL.setPower(-output); //backwards
                mDrive.BL.setPower(-output); //backwards
                mDrive.FR.setPower(0); //forwards
                mDrive.BR.setPower(-output); //forwards
            }
        }
        mDrive.freeze();
    }

}
