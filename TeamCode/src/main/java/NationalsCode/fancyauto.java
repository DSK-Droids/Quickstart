package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "fancyauto (Blocks to Java)")
public class fancyauto extends LinearOpMode {

    private DcMotor Upster;
    private DcMotor Extendo;
    private Servo Dumpster;
    private Servo gripster2;
    private Servo gripster;
    private Servo flipster;
    private DcMotor RF;
    private DcMotor RB;
    private DcMotor LB;
    private DcMotor LF;
    private Servo Swingster;
    double Ticks_per_cm;
    double maxSpeed;
    double UpsterPos;
    BNO055IMU.Parameters imuParameters;
    Orientation angles;
    Acceleration gravity;
    float steering;
    private BNO055IMU imu;
    double ExtendPos;
    double RoboAngle;



    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        gripster2 = hardwareMap.get(Servo.class, "gripster2");
        gripster = hardwareMap.get(Servo.class, "gripster");
        //flipster = hardwareMap.get(Servo.class, "flipsterAsServo");
        RF = hardwareMap.get(DcMotor.class, "fr");
        RB = hardwareMap.get(DcMotor.class, "br");
        LB = hardwareMap.get(DcMotor.class, "bl");
        LF = hardwareMap.get(DcMotor.class, "fl");
        imu = hardwareMap.get(BNO055IMU.class, "imuAsBNO055IMU");
        Upster = hardwareMap.get(DcMotor.class, "upster");
        Extendo = hardwareMap.get(DcMotor.class, "extendo");
        Dumpster = hardwareMap.get(Servo.class, "dumpster");
        Swingster = hardwareMap.get(Servo.class, "swingster");
        flipster = hardwareMap.get(Servo.class, "flipster");
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        float steering;
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        ExtendPos = Extendo.getCurrentPosition();

        waitForStart();


        if (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            steering = angles.firstAngle / 46;
            RoboAngle = angles.firstAngle;
            // Put run blocks here.
            RF.setDirection(DcMotor.Direction.FORWARD);
            RB.setDirection(DcMotor.Direction.FORWARD);
            LB.setDirection(DcMotor.Direction.REVERSE);
            LF.setDirection(DcMotor.Direction.REVERSE);
            Ticks_per_cm = 537.7 / (10.4 * Math.PI);

            Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Upster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Upster.setDirection(DcMotorSimple.Direction.FORWARD);
            //Upster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            UpsterPos = Upster.getCurrentPosition();


            drive_cm(-43, 0.5);
            drop();
            undrop();
            drive_cm(6, 0.5);
            strafetest(-24, 0.5);
            rotate_degrees(45+32., 0.5);
            intake();
            rotate_degrees(-107, 0.5);
            strafetest(18,0.5);
            fancy(300,0);
            drive_cm(-8,0.5);


            drop();
            undrop();
            sleep(1000);















































        }
    }


    private void strafetest(int cm, double power) {
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setTargetPosition(-(int) (LB.getCurrentPosition() + cm * Ticks_per_cm));
        LF.setTargetPosition((int) (LF.getCurrentPosition() + cm * Ticks_per_cm));
        RB.setTargetPosition((int) (RB.getCurrentPosition() + cm * Ticks_per_cm));
        RF.setTargetPosition(-(int) (RF.getCurrentPosition() + cm * Ticks_per_cm));
        LB.setPower(-power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(-power);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy()) {
        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void drive_cm(int cm, double power) {
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setTargetPosition((int) (LB.getCurrentPosition() + cm * Ticks_per_cm));
        LF.setTargetPosition((int) (LF.getCurrentPosition() + cm * Ticks_per_cm));
        RB.setTargetPosition((int) (RB.getCurrentPosition() + cm * Ticks_per_cm));
        RF.setTargetPosition((int) (RF.getCurrentPosition() + cm * Ticks_per_cm));
        LB.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        RF.setPower(power);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy()) {
        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void strafeL_cm(int cm, double power) {
        LB.setTargetPosition((int) (LB.getCurrentPosition() + cm * Ticks_per_cm));
        LF.setTargetPosition(-(int) (LF.getCurrentPosition() + cm * Ticks_per_cm));
        RB.setTargetPosition(-(int) (RB.getCurrentPosition() + cm * Ticks_per_cm));
        RF.setTargetPosition((int) (RF.getCurrentPosition() + cm * Ticks_per_cm));
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setPower(power);
        LF.setPower(-power);
        RB.setPower(-power);
        RF.setPower(power);
        while (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy()) {
        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method will rotate the robot by a specified angle (degrees) using the IMU.
    private void rotate_degrees(double targetAngle, double power) {
        // Reset encoders and set the robot in run mode
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor modes to run without position target (since we're using IMU feedback)
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU (assuming you have a method to get current angle)
        double startAngle = getCurrentAngle();  // You need to get the current IMU angle

        // Normalize the target angle to be within -180 to 180 degrees (angle wrapping)
        double targetAngleNormalized = normalizeAngle(startAngle + targetAngle);

        // Set motor power for rotation (positive for clockwise, negative for counterclockwise)
        double rotationPower = (targetAngle > 0) ? power : -power;

        // Rotate until the robot reaches the target angle
        while (Math.abs(normalizeAngle(getCurrentAngle()) - targetAngleNormalized) > 1) {  // Small threshold to account for sensor noise
            if (targetAngle > 0) {
                // Rotate clockwise
                LB.setPower(rotationPower);
                LF.setPower(rotationPower);
                RB.setPower(-rotationPower);
                RF.setPower(-rotationPower);
            } else {
                // Rotate counterclockwise
                LB.setPower(-rotationPower);
                LF.setPower(-rotationPower);
                RB.setPower(rotationPower);
                RF.setPower(rotationPower);
            }

            // Optionally, update telemetry to track rotation progress
            telemetry.addData("Current Angle", getCurrentAngle());
            telemetry.addData("Target Angle", targetAngleNormalized);
            telemetry.update();
        }

        // Stop all motors after reaching the target angle
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

        // Optionally, switch to run using encoders mode after rotation
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper method to normalize the angle within the range -180 to 180 degrees
    private double normalizeAngle(double angle) {
        // Normalize angle to the range -180 to 180 degrees
        while (angle <= -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    // Helper method to get the current IMU angle
    private double getCurrentAngle() {
        // Assuming your robot has an IMU object called imu, you need to get the current orientation:
        // Note: This is pseudo code; please adapt it to your IMU setup
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;  // Assuming `firstAngle` gives the heading in degrees
    }

    private void drop() {


            //Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Upster.setTargetPosition(-850);
            Upster.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Upster.setPower(-1);
            sleep(500);
            Dumpster.setPosition(0.6);
            sleep(2000);
            gripster2.setPosition(0.95);
            sleep(500);



    }


    private void undrop() {


            sleep(500);
            Upster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Upster.setTargetPosition(-5);
            Upster.setPower(0.5);
            sleep(200);
            Dumpster.setPosition(0);



    }

    private void intake() {

        Extendo.setTargetPosition(450);
        Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Extendo.setPower(-1);
        sleep(1000);

        Swingster.setPosition(0.13);
        sleep(500);
        gripster.setPosition(0.5);
        flipster.setPosition(0);
        sleep(1500);

        gripster.setPosition(1);
        sleep(1000);
        flipster.setPosition(1);
        Swingster.setPosition(1);
        sleep(1000);

        Extendo.setTargetPosition(0);
        Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Extendo.setPower(1);
        sleep(1000);

        flipster.setPosition(0.73);
        sleep(200);
        gripster2.setPosition(0.6);
        sleep(300);
        gripster.setPosition(0.5);
        sleep(200);





    }

    private void  fancy(int time, double power) {

        LB.setPower(0.3);
        LF.setPower(0.3);
        RB.setPower(-0.3);
        RF.setPower(-0.3);
        sleep(time);



    }

}







