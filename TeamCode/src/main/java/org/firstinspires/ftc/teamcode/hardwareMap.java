package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hardwareMap {

    HardwareMap hardwareMap;

    public DcMotorEx FL; //port 0
    public DcMotorEx FR; //port 1
    public DcMotorEx BL; //port 2
    public DcMotorEx BR; //port 3
    public DcMotorEx Filler1, Filler2;
    public DcMotor Laft; //port 0 ExpansionHub 40:1
    public CRServo InOut; //port _
    public DcMotor Duckie; //port _
    public DcMotor.RunMode RUN_USING_ENCODER;
    public DcMotorEx  liftOne;
    public DcMotorEx liftTwo;
    public CRServo  vexClaw;

    public BNO055IMU imu;

    public void init(HardwareMap h) {
        hardwareMap = h;
        BR = (DcMotorEx)hardwareMap.get( DcMotorEx.class, "rightBack");
        FR = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rightFront");
        FL = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "leftFront");
        BL = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "leftBack");

        liftOne     = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "liftOne");
        liftTwo     = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "liftTwo");
        vexClaw     = hardwareMap.crservo.get("vexClaw");

        //BR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //liftOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //liftTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

      //  Laft = hardwareMap.get(DcMotor.class, "L1"); //lift motor imo
      //  InOut = hardwareMap.get(CRServo.class, "IO1"); //intake\outtake motor
       // Duckie = hardwareMap.get(DcMotor.class, "D1"); //duck motor
      //  Filler1 = hardwareMap.get(DcMotorEx.class, "F1");
      //  Filler2 = hardwareMap.get(DcMotorEx.class, "F2");



       /* Laft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duckie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Filler1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Filler2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void resetEncoders()
    {
     //   Laft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // Duckie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // Filler1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // Filler2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // Laft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // Duckise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  Filler1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // Filler2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getEncoderAvg() {
        double output = 0;
        int encoderCount = 0;
        boolean[] encoderIsNotPluggedIn = new boolean[4];

      /*  if (Math.abs(Laft.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(Laft.getCurrentPosition());
        } else encoderIsNotPluggedIn[0] = true;

       /* if (Math.abs(Duckie.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(Duckie.getCurrentPosition());
        } else encoderIsNotPluggedIn[1] = true;

        if (Math.abs(Filler1.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(Filler1.getCurrentPosition());
        } else encoderIsNotPluggedIn[2] = true;

        if (Math.abs(Filler2.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(Filler2.getCurrentPosition());
        } else encoderIsNotPluggedIn[3] = true;*/

        if (encoderCount == 0)
            return 0;
        else
            return output/encoderCount;
    }

    public void freeze(){
       /* FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);*/
        //Laft.setPower(0);
       // Duckie.setPower(0);
    }
}