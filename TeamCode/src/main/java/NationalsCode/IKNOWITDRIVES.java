package NationalsCode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IKNOWITDRIVES", group="Linear Opmode")
public class IKNOWITDRIVES extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LFDrive;
    private DcMotor RFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;
    private DcMotor Extendo;
    private Servo Swingster;
    private Servo Gripster;
    private Servo Gripster2;
    private Servo Flipster;
    private Servo Dumpster;
    private DcMotor Upster;
    double ServoAngle;
    boolean Triggerup;
    boolean TriggerDown;
    boolean Backtouch;
    boolean Fronttouch;
    boolean transferDone=true;
    private RevTouchSensor MagnetFront;
    private RevTouchSensor MagnetBack;
    double  ExtendPos;
    double TransparentPower;

    double UpsterPos;
    boolean UpgoUp;
    boolean OutGoIn;
    boolean UpGoDown;

    boolean Macro;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize an ElapsedTime object to keep track of timing for non-blocking delays


        ServoAngle = 1;
        ExtendPos = 0;
        TransparentPower =1;
        LBDrive = hardwareMap.get(DcMotor.class, "bl");
        LFDrive = hardwareMap.get(DcMotor.class, "fl");
        RBDrive = hardwareMap.get(DcMotor.class, "br");
        RFDrive = hardwareMap.get(DcMotor.class, "fr");
        Extendo = hardwareMap.get(DcMotor.class, "extendo");
        Swingster = hardwareMap.get(Servo.class, "swingster");
        Gripster = hardwareMap.get(Servo.class, "gripster");
        Flipster = hardwareMap.get(Servo.class, "flipster");
        MagnetFront = hardwareMap.get(RevTouchSensor.class, "fronttouch");
        MagnetBack = hardwareMap.get(RevTouchSensor.class, "backtouch");
        Upster = hardwareMap.get(DcMotor.class, "upster");
        RBDrive.setDirection(DcMotor.Direction.REVERSE);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        Extendo.setDirection(DcMotor.Direction.FORWARD);
       // LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Upster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Upster = hardwareMap.get(DcMotor.class, "upster");
        Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Upster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Dumpster = hardwareMap.get(Servo.class, "dumpster");

        Gripster2 = hardwareMap.get(Servo.class, "gripster2");
        Gripster2.setPosition(1);
        Dumpster.setPosition(0);
        ElapsedTime flipsterTimer = new ElapsedTime();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            ExtendPos = Extendo.getCurrentPosition();
            UpsterPos = Upster.getCurrentPosition();
            OutGoIn = false;
            Macro = false;


            if (UpgoUp && UpsterPos > -800) {

                Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Upster.setTargetPosition(-800);
                Upster.setPower(-0.5);
            } else if (UpGoDown && UpsterPos < -5) {

                Upster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Upster.setTargetPosition(-1);
                Upster.setPower(0.5);
            } else {
                Upster.setPower(0);
            }


            if (gamepad2.a) {
                Flipster.setPosition(0);
            }

            if (gamepad2.y) {
                Flipster.setPosition(1);
            }

            if (gamepad2.right_trigger > 0.1) {
                Gripster.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                Gripster.setPosition(0.5);
            }


            if (gamepad2.left_stick_x > 0.1 && ServoAngle <= 1) {
                ServoAngle = ServoAngle + gamepad2.left_stick_x / 100;
            } else if (gamepad2.left_stick_x < -0.1 && ServoAngle >= 0) {
                ServoAngle = ServoAngle + gamepad2.left_stick_x / 100;
            }

            if (gamepad2.right_stick_y < -0.1) {
                Extendo.setTargetPosition(450);
                Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extendo.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0.1) {
                Extendo.setTargetPosition(0);
                Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extendo.setPower(gamepad2.right_stick_y);

            } else if (!OutGoIn) {
                Extendo.setPower(0);
            }

            if (gamepad1.dpad_left) {
                Dumpster.setPosition(1);
            } else if (gamepad1.dpad_right) {
                Dumpster.setPosition(0);
            }

            if (gamepad1.x) {
                Gripster2.setPosition(0.75);
            } else if (gamepad1.b) {
                Gripster2.setPosition(0.95);
            }

            //Move servo arm based on trigger
            if (gamepad2.left_trigger > 0.2 || transferDone==false) {
                if (transferDone==true){
                    flipsterTimer.reset();
                    transferDone = false;
                }

                // Start the Flipster action and other tasks
                Flipster.setPosition(1);
                //TODO: Define positions for transfer, sample, and specimen placements
                //
                // Check the elapsed time
                double timeElapsed = flipsterTimer.seconds();
                telemetry.addData("Flipster Timer", timeElapsed);
                telemetry.update();

                if (timeElapsed < 0.1) {
                    // Execute action between 50ms and 100ms (converted to seconds)
                    ServoAngle = 1;
                    Swingster.setPosition(ServoAngle);
                }

                else if (timeElapsed >= 0.5 && timeElapsed < 2.9) {
                    // After 500ms but before 1300ms, set Extendo to move
                    OutGoIn = true;
                    if (ExtendPos > 0) {
                        Extendo.setTargetPosition(-25);
                        Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Extendo.setPower(1);

                    OutGoIn = false;
                    }
                }

                /*else if (timeElapsed >= 2.6 && timeElapsed < 3 ){
                    // Between 1300ms and 1800ms, slow down Flipster
                   // Flipster.setPosition(0.8);
                }/*
                    */
                else if (timeElapsed >= 3 && timeElapsed < 3.3) {
                    // Between 1800ms and 2100ms, reset Gripsters
                    Flipster.setPosition(0.73);
                    sleep(200);
                    Gripster2.setPosition(0.75);

                }

                else if (timeElapsed >= 3.6 ) {
                    // Between 1800ms and 2100ms, reset Gripsters

                    Gripster.setPosition(0.5);
                    transferDone = true;
                }
            }

            if (gamepad2.left_bumper) {
                Flipster.setPosition(0.8);
            }


            if (gamepad2.dpad_up) {
                Upster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                UpGoDown = false;
                UpgoUp = true;
                Dumpster.setPosition(0.7);

            }else if (gamepad2.dpad_down) {
                //Upster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Dumpster.setPosition(0);
                UpgoUp = false;
                UpGoDown = true;
            }










            if (gamepad1.left_trigger > 0.2) {
                TransparentPower = 0.5;
            } else {
                TransparentPower = 1;
            }



            double LSY = gamepad1.left_stick_y;
            double LSX = gamepad1.left_stick_x;
            double RSX = gamepad1.right_stick_x;
            double powerScale = Math.max(1.0, (Math.abs(LSY) + Math.abs(LSX) + Math.abs(RSX)));

            double LFPower = ((LSY) + (-RSX) + (-LSX)) / powerScale;
            double RFPower = ((LSY) + (RSX) + (LSX)) / powerScale;
            double LBPower = ((LSY) + (-RSX) + (LSX)) / powerScale;
            double RBPower = ((LSY) + (RSX) + (-LSX)) / powerScale;

            LFDrive.setPower(LFPower * (TransparentPower));
            RFDrive.setPower(RFPower * (TransparentPower));
            LBDrive.setPower(LBPower * (TransparentPower));
            RBDrive.setPower(RBPower * (TransparentPower));

            Swingster.setPosition(ServoAngle);


                

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "LF (%.2f), LB (%.2f), RB (%.2f), RF (%.2f)", LFPower, LBPower, RBPower, RFPower);
            telemetry.addData("transparent power????", TransparentPower);
            telemetry.addData("is down pressed?", Triggerup);
            telemetry.addData("servo anglllleee???????", UpsterPos);
            telemetry.addData("timerr???????", flipsterTimer);
            telemetry.update();

        }
    }
}