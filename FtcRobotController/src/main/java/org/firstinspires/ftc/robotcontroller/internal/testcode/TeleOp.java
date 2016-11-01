package org.firstinspires.ftc.robotcontroller.internal.testcode;
import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Student on 10/11/2016.
 */
public class TeleOp extends OpMode {

    Servo pointyL;
    Servo pointyR;

    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotor Flapper1;
    DcMotor Flapper2;


    DcMotor topWheel;
    DcMotor bottomWheel;

    final double TOP_ON_POSITION = 1.0;
    final double TOP_OFF_POSITION = 0.5;
    final double BOTTOM_ON_POSITION = 1.0;
    final double BOTTOM_OFF_POSITION = 0.5;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

                rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Flapper1 = hardwareMap.dcMotor.get("Flapper");

        topWheel = hardwareMap.dcMotor.get("top_wheel");
        bottomWheel = hardwareMap.dcMotor.get("bottom_wheel");

        pointyL = hardwareMap.servo.get("pointyL");
        pointyR = hardwareMap.servo.get("pointyR");
    }
    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            pointyL.setPosition(1.0);
        }
        pointyL.setPosition(0);

        if (gamepad1.right_bumper) {
            pointyR.setPosition(1.0);
        }
        pointyR.setPosition(0);

        Gamepad gamepad1 = new Gamepad();
        float xValue = gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        float leftPower = yValue + xValue;
        float rightPower = yValue + xValue;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (gamepad1.y) {
            Flapper1.setPower(0.1);
            Flapper2.setPower(0.1);
        } else if (gamepad1.b) {
            Flapper1.setPower(-0.1);
            Flapper2.setPower(-0.1);
        }
        Flapper1.setPower(0);
        Flapper2.setPower(0);

        if (gamepad1.x) {
            topWheel.setPower(TOP_ON_POSITION);
            bottomWheel.setPower(BOTTOM_ON_POSITION);
        }
        else {
            topWheel.setPower(TOP_OFF_POSITION);
            bottomWheel.setPower(BOTTOM_OFF_POSITION);
        }
    }
}
