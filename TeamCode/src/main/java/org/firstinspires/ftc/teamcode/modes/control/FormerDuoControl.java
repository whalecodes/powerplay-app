package org.firstinspires.ftc.teamcode.modes.control;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.TouchSensor;


// TODO - rework
// last year's opmode adapted to work for this year
@Disabled
@TeleOp(name = "Freight Frenzy Test Tele", group = "")
public class FormerDuoControl extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor Arm;
    private Servo Gripper1;
    private Servo Gripper2;
    //  private Servo Gripper3;
    private Servo Gripper4;
    private TouchSensor touchSensor;

    private boolean keepArmRaised = false;
    private boolean keepArmLowered = false;

    @Override
    public void init()
    {

        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");
        Gripper1 = hardwareMap.servo.get("S1");
        Gripper2 = hardwareMap.servo.get("S2");
        //Gripper3 = hardwareMap.servo.get("S3");
        Gripper4 = hardwareMap.servo.get("S4");
        Arm = hardwareMap.dcMotor.get("arm");
        //Gripper4.setPosition(0);
        //Gripper4.scaleRange(0, 1);

        touchSensor = hardwareMap.touchSensor.get("TS");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * (1.4);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4;
        double rightX = 1 * gamepad1.right_stick_x;
        /*if(gamepad1.left_bumper) r /= 2;
        if(gamepad1.right_bumper) r /= 4;*/
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        if (Math.abs(v1) > 1 || Math.abs(v3) > 1 || Math.abs(v2) > 1 || Math.abs(v4) > 1 ) {
            double max = 0;
            max = Math.max(Math.abs(v1), Math.abs(v3));
            max = Math.max(Math.abs(v2), max);
            max = Math.max(Math.abs(v4), max);

            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }
        if (gamepad1.right_trigger > 0){
            frontLeft.setPower(.5 * v1);
            frontRight.setPower(-.5 * v2);
            backLeft.setPower(.5 * v3);
            backRight.setPower(.5 * v4);
        }
        else{
            frontLeft.setPower(1 * v1);
            frontRight.setPower(-1 * v2);
            backLeft.setPower(1 * v3);
            backRight.setPower(1 * v4);
        }

        double armPower = gamepad2.left_stick_y;
        boolean enableKeepArmRaised = gamepad2.right_bumper;
        boolean enableKeepArmLowered = gamepad2.left_bumper;

        if (enableKeepArmRaised) {
            keepArmRaised = true;
        } else if (enableKeepArmLowered) {
            keepArmLowered = true;
        } else if (armPower != 0) {
            keepArmRaised = false;
            keepArmLowered = false;
        }

        if (keepArmLowered) armPower = 0.8;
        if (keepArmRaised) armPower = -1;

        if (armPower > 0.8) armPower = 0.8;
        if (armPower == 0) armPower = -0.1;

        Arm.setPower(armPower);

        telemetry.addLine("Arm Position: " + Arm.getCurrentPosition() + ", " + Arm.getTargetPosition());

        telemetry.addLine("Arm Power: " + armPower);

        if (gamepad2.y) {
            Gripper1.setPosition(0);

            Gripper2.setPosition(1);
        }
        if (gamepad2.a || touchSensor.isPressed()) {
            Gripper2.setPosition(0);

            Gripper1.setPosition(1);
        }


        if (!(gamepad2.y)) {

            if (gamepad2.y) {
                //Gripper2.setDirection(DcMotorSimple.Direction.FORWARD);
                //Gripper2.setPosition(0);
                //Gripper1.setDirection(DcMotorSimple.Direction.FORWARD);
                //Gripper1.setPosition(1);
            } else if (gamepad2.a) {
                //Gripper2.setDirection(DcMotorSimple.Direction.REVERSE);
                //Gripper2.setPosition(.5);
                //Gripper1.setDirection(DcMotorSimple.Direction.REVERSE);
                //Gripper1.setPosition(.5);

            } else {
                //Gripper2.setPosition(-.5);
                //Gripper1.setPosition(.5);
            }

        }

        //Arm.setPower(.3 * gamepad2.right_stick_y);
        //if（gamepad1.right_trigger > 0)
        {
            // Arm.setPower(.9);
        }


        //if(gamepad2.right_stick_y >0){
        // Return.setPower(gamepad2.right_stick_y);
        // }
        //  else{BabySlider.setPower(.8* gamepad2.right_stick_y);
        //    Return.setPower(.75* gamepad2.right_stick_y);

        //  }
        //Gripper4.scaleRange(0, 1);


        if(gamepad2.x){

            //Gripper4.setDirection(DcMotorSimple.Direction.FORWARD);

            Gripper4.setDirection(Servo.Direction.FORWARD);
            Gripper4.setPosition(1);



        } else if (gamepad2.b){

            Gripper4.setDirection(Servo.Direction.REVERSE);
            Gripper4.setPosition(.85);
        }

        telemetry.update();


        //else {
        //Gripper4.setPower(0);


        //} else if (gamepad2.y) {
        //Gripper.setPosition(0);
        //}

        //if (gamepad2.left_stick_x != 0) {

        //    float value = gamepad2.left_stick_x;

        //    float middle = 0.5;

        //    Gripper4.setPosition(middle + value);

        //    Gri

    }



}
