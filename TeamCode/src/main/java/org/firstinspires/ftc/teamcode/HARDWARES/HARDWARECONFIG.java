package org.firstinspires.ftc.teamcode.HARDWARES;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.HARDWARES.UPPERPOWERFILE.upperpowerbound;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SUBS.PowerSUB;
import org.firstinspires.ftc.teamcode.SUBS.SERVOSUB;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.initMovement.InitMovementController;

import java.util.List;

public class HARDWARECONFIG {
    boolean slowmode = false;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;
    public SERVOSUB servosub = null;
    public PowerSUB powersub = null;
    //public org.firstinspires.ftc.teamcode.SUBS.ARMSUB armSub = null;
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    // Limelight3A limelight = null;
//    DcMotor gunmotorR = null;
//    DcMotor gunmotorL = null;
//    DcMotor intakeR = null;
//    DcMotor intakeL = null;
    double heading = 0;
    double distance = 0;
    Pose2d startPose = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    MecanumDrive drive = null;
    FtcDashboard dash = null;
    double x = 0;
    double y = 0;
    double indicator = 0;
//    private LED redLed;
//    private LED greenLed;
//    private LED redLed1;
//    private LED greenLed1;
//    private LED redLed2;
//    private LED greenLed2;


    double color = 0;

    //private VisionPortal visionPortal;
    //private AprilTagProcessor aprilTag;
    private InitMovementController imc = null;

    ElapsedTime elapsedTime = null;

    public HARDWARECONFIG(LinearOpMode om, HardwareMap hwmap, Boolean auto) {
        initrobot(hwmap, om, auto);

        powersub = new PowerSUB(hwmap);
        servosub = new SERVOSUB(hwmap);

    }

    Action t = null;

    void initrobot(HardwareMap hwmap, LinearOpMode om, Boolean auto) {
        opMode = om;//
        telemetry = om.telemetry;
        imc = new InitMovementController(opMode.gamepad2,opMode.gamepad1);
        //clawsub = new CLAWSUB(hwmap);
        // armSub = new org.firstinspires.ftc.teamcode.SUBS.ARMSUB(hwmap, auto);
        frontLeftMotor = hwmap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hwmap.dcMotor.get("backLeftMotor");
        frontRightMotor = hwmap.dcMotor.get("frontRightMotor");
        backRightMotor = hwmap.dcMotor.get("backRightMotor");

        dash = FtcDashboard.getInstance();

//        gunmotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        gunmotorL.setDirection(DcMotorSimple.Direction.FORWARD);

        drive = new MecanumDrive(hwmap, (Pose2d) blackboard.getOrDefault(currentpose, new Pose2d(0, 0, 0)));

       // t = Turn(1.7);
//         limelight = hwmap.get(Limelight3A.class, "limelight");
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
//        gunmotorL.setDirection(DcMotorSimple.Direction.FORWARD);
//        gunmotorR.setDirection(DcMotorSimple.Direction.REVERSE);



//        aprilTag = new AprilTagProcessor.Builder()
//
//                // The following default settings are available to un-comment and edit as needed.
//                .setDrawAxes(true)
//                .setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
//
//                // == CAMERA CALIBRATION ==
//                // If you do not manually specify calibration parameters, the SDK will attempt
//                // to load a predefined calibration for your camera.
//                .setLensIntrinsics(572.066, 572.066, 327.923, 257.789)
//                // ... these parameters are fx, fy, cx, cy.
//
//
//                .build();
        //VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        //builder.setCamera(hwmap.get(WebcamName.class, "Webcam 1"));
        //builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
       // visionPortal = builder.build();
        //FtcDashboard.getInstance().startCameraStream(visionPortal, 60);

        elapsedTime = new ElapsedTime();
    }

//    public int getrandomization() {
//        //apriltags need to find the green 1 for first pos 2 for second pos 3 for third.
//        //21,22,23
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.id >= 21 && detection.id <= 23) {
//                return detection.id - 20;
//
//            }
//
//        }
//        return 0;
//    }

    public static String currentpose = "currentpose";

//    public void SetRedLED(boolean isOn) {
//        if (isOn) {
//            redLed.on();
//        } else {
//            redLed.off();
//        }
//    }
//    public void SetRedLED1(boolean isOn) {
//        if (isOn) {
//            redLed1.on();
//        } else {
//            redLed1.off();
//        }
//    }
//    public void SetRedLED2(boolean isOn) {
//        if (isOn) {
//            redLed2.on();
//        } else {
//            redLed2.off();
//        }
//    }
//
//    public void SetGreenLED(boolean isOn) {
//        if (isOn) {
//            greenLed.on();
//        } else {
//            greenLed.off();
//        }
//    }
//    public void SetGreenLED1(boolean isOn) {
//        if (isOn) {
//            greenLed1.on();
//        } else {
//            greenLed1.off();
//        }
//    }
//    public void SetGreenLED2(boolean isOn) {
//        if (isOn) {
//            greenLed2.on();
//        } else {
//            greenLed2.off();
//        }
//    }



//    public Action Turn(double angle) {
//        Scribe.getInstance().logData("here");
//        return drive.actionBuilder(drive.localizer.getPose())
//                .turnTo(angle).build();
//    }

    Action runningaction = null;

//    public void lockit() {
//        TelemetryPacket p = new TelemetryPacket();
//        double angle = getheadingfromAT();
//
//
////        Scribe.getInstance().logData(angle);
//        if (angle != 0) {
//            Action t = Turn(angle);
//
//
//            if (runningaction != null) {
////                runningaction.preview(p.fieldOverlay());
//                Actions.runBlocking(runningaction);
//                runningaction = null;
////                runningaction.run(p);
////                if (!runningaction.run(p)) {
////                    Scribe.getInstance().logData("true");
////                    runningaction = null;
////                }
//
//            } else {
//                runningaction = t;
//            }
//            dash.sendTelemetryPacket(p);
//        }
//    }

//    public double getrangefromAT() {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.id == 20 || detection.id == 24) {
//                return detection.ftcPose.range;
//            }
//
//        }
//        return -1;
//
//    }

    public void indicatormath() {
        if (heading <= 0) {
            indicator = 1;
        } else if (heading > -0.6) {
            indicator = 0;
        }
    }

//    public void lighton() {
//        if (indicator > 0) {
//            SetGreenLED(true);
//            SetRedLED(false);
//        } else if (indicator < 1) {
//            SetGreenLED(false);
//            SetRedLED(true);
//        }
//    }


//    public double getheadingfromAT() {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.id == 20 || detection.id == 24) {
//                if (detection.id == 20){
//                    return Math.PI/4;
//                }else{
//                    return (7*Math.PI)/4;
//                }
//            }
//
//        }
//        return 0;
//    }



    public void buildtelemetry() {
        telemetry.addData("slowmode", slowmode);
        telemetry.addData("heading", heading);
        telemetry.addData("distance", distance);
        telemetry.addData("Power", upperpowerbound);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("indicator", indicator);
        powersub.telemetry(telemetry);
        telemetry.update();
    }

    boolean touchpadwpressed = false;

    public void dobulk() {//
        imc.checkHasMovedOnInit();
        //heading = getheadingfromAT();
       // distance = getrangefromAT();
        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = opMode.gamepad1.right_stick_x;
        boolean touchpadpressed = opMode.gamepad1.touchpad;
        if (touchpadpressed && !touchpadwpressed) {
            slowmode = !slowmode;
        }
        touchpadwpressed = touchpadpressed;
        double slowmodemultiplier = 0.5;


        touchpadwpressed = touchpadpressed;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double multiplier = 1;
        if (slowmode) {
            multiplier = slowmodemultiplier;
        }
        double frontLeftPower = ((y + x + rx) / denominator) * multiplier;
        double backLeftPower = ((y - x + rx) / denominator) * multiplier;
        double frontRightPower = ((y - x - rx) / denominator) * multiplier;
        double backRightPower = ((y + x - rx) / denominator) * multiplier;
//        double gunmotorPower = Range.clip(opMode.gamepad1.right_trigger, -1, 1);
//        double gunmotorPowerL = gunmotorPower;


        double armpower = 0;


//
//! -0.65, -0.4 0.5, 0.4 FIX
        if (heading >= -0.65 && heading <= -0.4) {
            indicator = 1;
        } else if (heading >= 0.4 && heading <= 0.5) {
            indicator = 1;
        } else {
            indicator = 0;
        }



//        if (indicator == 1) {
//            SetGreenLED(false);
//            SetRedLED(true);
//        } else if (indicator == 0) {
//            SetGreenLED(true);
//            SetRedLED(false);
//        }



        if (opMode.gamepad1.left_bumper) {
            powersub.intakereverse();
        } else if (opMode.gamepad1.right_bumper) {
            powersub.intakeon();
        } else {
            powersub.intakeoff();
        }





        if (opMode.gamepad2.left_trigger > 0) {
            powersub.extrapower();
            powersub.intakeon();
        }else if (opMode.gamepad2.right_trigger > 0){
            powersub.power();
            powersub.intakeon();
        }else {
            powersub.gunidle();
        }

        if (opMode.gamepad2.dpad_up){
            servosub.SafetyON();
        }
        else if (opMode.gamepad2.dpad_down){
            servosub.Safetyoff();
        }



//        if (opMode.gamepad2.x){
//            servosub.lscoopdown();
//        }else {
//            servosub.lscoopup();
//        }
//        if (opMode.gamepad2.b){
//            servosub.rscoopdown();
//        }else {
//            servosub.rscoopup();
//        }
//        if (opMode.gamepad2.a){
//            servosub.rscoopup();
//            servosub.lscoopup();
//        }






        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (imc.hasMovedOnInit()){
            servosub.update();
            powersub.update();
        }
        //armSub.update();

        buildtelemetry();

    }
}

