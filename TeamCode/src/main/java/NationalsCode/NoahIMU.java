package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "NoahIMU (Blocks to Java)")
public class NoahIMU extends LinearOpMode {

    private DcMotor RB;
    private DcMotor RF;
    private BNO055IMU imu;
    private DcMotor LF;
    private DcMotor LB;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double maxSpeed;
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        float steering;

        RB = hardwareMap.get(DcMotor.class, "RBAsDcMotor");
        RF = hardwareMap.get(DcMotor.class, "RFAsDcMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imuAsBNO055IMU");
        LF = hardwareMap.get(DcMotor.class, "LFAsDcMotor");
        LB = hardwareMap.get(DcMotor.class, "LBAsDcMotor");

        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        // Create new IMU Parameters object.
        maxSpeed = 0.75;
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Get absolute orientation
                // Get acceleration due to force of gravity.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                // Display orientation info.
                steering = angles.firstAngle / 46;
                if (angles.firstAngle < 0) {
                    LF.setPower(maxSpeed + steering);
                    LB.setPower(maxSpeed + steering);
                    RF.setPower(maxSpeed);
                    RB.setPower(maxSpeed);
                } else if (angles.firstAngle > 0) {
                    LF.setPower(maxSpeed);
                    LB.setPower(maxSpeed);
                    RF.setPower(maxSpeed - steering);
                    RB.setPower(maxSpeed - steering);
                } else {
                    LF.setPower(maxSpeed);
                    LB.setPower(maxSpeed);
                    RF.setPower(maxSpeed);
                    RB.setPower(maxSpeed);
                }
            }
        }
        // Express acceleration as m/s^2.
    }
}