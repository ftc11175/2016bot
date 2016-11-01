package org.firstinspires.ftc.robotcontroller.internal.testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.Timer;

/**
 * Created by michael on 10/24/2016.
 */

public class Autonomous extends OpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.14);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo pointyL;
    Servo pointyR;

    ColorSensor color;

    UltrasonicSensor sonic;

    OpticalDistanceSensor ODS;

    GyroSensor Gyro;

    boolean goalReached;

    int a;
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        pointyL = hardwareMap.servo.get("pointyL");
        pointyR = hardwareMap.servo.get("pointyR");

        color = hardwareMap.colorSensor.get("color");

        sonic = hardwareMap.ultrasonicSensor.get("sonic");

        a = (int) 12.5;

        ODS = hardwareMap.opticalDistanceSensor.get("ODS");

        Gyro = hardwareMap.gyroSensor.get("Gyro");

    }


        @Override
        public void loop () {

            if (!Gyro.isCalibrating()) {
                if ((Gyro.getHeading() >= 90))
                    if ((Gyro.getHeading() <= 90)) {
                        goalReached = true;
                    }
                if (goalReached) {
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                }
                if (!goalReached) {
                    rightMotor.setPower(0.175);
                    leftMotor.setPower(-0.175);
                }
                telemetry.addData("gyro", "X:" + Gyro.rawX() / 100 + " Y:" + Gyro.rawY() / 100 + " Z:" + Gyro.rawZ() / 100);
                telemetry.addData("gyroHead", String.valueOf(Gyro.getHeading()));

                double reflectance = ODS.getLightDetected();

                if (reflectance >= 0.25) {
                    rightMotor.setPower(-0.2);
                    leftMotor.setPower(0);
                } else {
                    rightMotor.setPower(0);
                    leftMotor.setPower(-0.2);
                    telemetry.addData("Reflectance", reflectance);
                }

                TopOneOne();
                TopTwoOne();
                TopThreeOne();
                TopFinalOne();
                TopOneTwo();
                TopTwoTwo();
                TopThreeTwo();
                TopFourTwo();
                TopFiveTwo();
                TopFinalTwo();
            }
        }


        public void TopOneOne () {

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(24 - a);
            rightMotor.setTargetPosition(24 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopTwoOne () {

            TurnRight();

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(48 - a);
            rightMotor.setTargetPosition(48 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {

            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopThreeOne () {

            TurnLeft();

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(36 - a);
            rightMotor.setTargetPosition(36 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {

            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        public void TopFinalOne () {

            TurnRight();


            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(25 - a);
            rightMotor.setTargetPosition(25 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //implement line following
        }
        public void TopOneTwo () {

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(24 - a);
            rightMotor.setTargetPosition(24 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopTwoTwo () {

            TurnRight();

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(48 - a);
            rightMotor.setTargetPosition(48 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {

            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopThreeTwo () {

            TurnLeft();

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(36 - a);
            rightMotor.setTargetPosition(36 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {

            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopFourTwo () {

            TurnRight();


            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(24 - a);
            rightMotor.setTargetPosition(24 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopFiveTwo () {


            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(48 - a);
            rightMotor.setTargetPosition(48 - a);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void TopFinalTwo () {

            TurnRight();

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setTargetPosition(2);
            rightMotor.setTargetPosition(2);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            DriveForward();

            while (leftMotor.isBusy() && rightMotor.isBusy()) ;
            {
            }

            StopDriving();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void StopDriving () {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
        public void DriveForward () {
            rightMotor.setPower(1);
            leftMotor.setPower(1);
        }
        public void TurnRight () {
            rightMotor.setPower(-1);
            leftMotor.setPower(1);


        }
        public void TurnLeft () {
            rightMotor.setPower(1);
            leftMotor.setPower(-1);
        }
}

