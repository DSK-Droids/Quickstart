//package NationalsCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//@Disabled
//public class RobotHardware {
//    public DcMotor FR = null;
//    public DcMotor FL = null;
//    public DcMotor BR = null;
//    public DcMotor BL = null;
//
//    public DcMotor intakeExtension = null;
//    public DcMotor placementExtension = null;
//    public Servo placementGrip = null;
//    public Servo placementSwivel = null;
//    public Servo intakeGrip = null;
//    public Servo intakeSwivel = null;
//
//    public Hardware(HardwareMap hwmap){
//        initialize(hwmap);
//    }
//
//    private initialize(HardwareMap hwmap){
//        hardwareMap=hwmap;
//
//        FL = hardwareMap.get(DcMotor.class, "fl");
//        FR = hardwareMap.get(DcMotor.class, "fr");
//        BL = hardwareMap.get(DcMotor.class, "bl");
//        BR = hardwareMap.get(DcMotor.class, "br");
//        intakeExtension = hardwareMap.get(DcMotor.class, "intakeExtension");
//        placementExtension = hardwareMap.get(DcMotor.class, "placementExtension");
//        placementGrip = hardwareMap.get(Servo.class, "placementGrip");
//        placementSwivel = hardwareMap.get(Servo.class, "placementSwivel");
//        intakeGrip = hardwareMap.get(Servo.class, "intakeGrip");
//        intakeSwivel = hardwareMap.get(Servo.class, "intakeSwivel");
//
//    }
//
//}
