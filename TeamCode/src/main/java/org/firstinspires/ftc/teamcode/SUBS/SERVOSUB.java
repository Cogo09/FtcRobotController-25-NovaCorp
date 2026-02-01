package org.firstinspires.ftc.teamcode.SUBS;

import static org.firstinspires.ftc.teamcode.UTILITIES.UTIL.setpose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HARDWARES.SERVOUTIL;

import java.util.List;

public class SERVOSUB {
    private final Servo leftelevatorservo;
    private final Servo middleelevatorservo;
    private final Servo rightelevatorservo;
    private final Servo leftscooperservo;
    private final Servo rightscooperservo;
//    private final Servo bootkickerservo;


    public enum LeftELEState {UP, DOWN, IDLE}

    private LeftELEState LeftELEStateVar = LeftELEState.IDLE;

    public void LELEup() {
        LeftELEStateVar = LeftELEState.UP;
    }

    public void LELEdown() {
        LeftELEStateVar = LeftELEState.DOWN;
    }

    public void LELEIDLE() {
        LeftELEStateVar = LeftELEState.IDLE;
    }

    public enum RightELEState {UP, DOWN, IDLE}

    private RightELEState RightELEStateVar = RightELEState.IDLE;

    public void RELEup() {
        RightELEStateVar = RightELEState.UP;
    }

    public void RELEdown() {
        RightELEStateVar = RightELEState.DOWN;
    }

    public void RELEIDLE() {
        RightELEStateVar = RightELEState.IDLE;
    }

    public enum MiddleELEState {UP, DOWN,SLIGHT, IDLE}

    private MiddleELEState MiddleELEStateVar = MiddleELEState.IDLE;

    public void MELEup() {
        MiddleELEStateVar = MiddleELEState.UP;
    }

    public void MELEdown() {
        MiddleELEStateVar = MiddleELEState.DOWN;
    }
    public void MELEslight(){MiddleELEStateVar = MiddleELEState.SLIGHT;}

    public void MELEIDLE() {
        MiddleELEStateVar = MiddleELEState.IDLE;
    }

    public enum RightScooperState {UP,DOWN,IDLE}

    private RightScooperState RightScooperStateVar = RightScooperState.IDLE;

    public void rscoopdown(){RightScooperStateVar = RightScooperState.DOWN;}

    public void rscoopup(){RightScooperStateVar = RightScooperState.UP;}

    public void rscoopidle(){RightScooperStateVar = RightScooperState.IDLE;}

    public enum LeftScooperState {UP,DOWN,IDLE}

    private LeftScooperState LeftScooperStateVar = LeftScooperState.IDLE;

    public void lscoopdown(){LeftScooperStateVar = LeftScooperState.DOWN;}

    public void lscoopup(){LeftScooperStateVar = LeftScooperState.UP;}

    public void lscoopidle(){LeftScooperStateVar = LeftScooperState.IDLE;}

    public enum BootState {UP, DOWN, IDLE}

//    private BootState BootStateVar = BootState.IDLE;
//
//    public void Bootup() {
//        BootStateVar = BootState.UP;
//    }
//
//    public void Bootdown() {
//        BootStateVar = BootState.DOWN;
//    }
//
//    public void BootIDLE() {
//        BootStateVar = BootState.IDLE;
//    }



    //this is where you put all enums and variables
    public SERVOSUB(HardwareMap hwMap) {
        leftelevatorservo = hwMap.get(Servo.class, "leftelevatorservo");
        middleelevatorservo = hwMap.get(Servo.class, "middleelevatorservo");
        rightelevatorservo = hwMap.get(Servo.class, "rightelevatorservo");
        leftscooperservo = hwMap.get(Servo.class,"leftscooperservo");
        rightscooperservo = hwMap.get(Servo.class,"rightscooperservo");
//        bootkickerservo = hwMap.get(Servo.class,"bootkickerservo");
        leftelevatorservo.setDirection(Servo.Direction.FORWARD);
        middleelevatorservo.setDirection(Servo.Direction.FORWARD);
        rightelevatorservo.setDirection(Servo.Direction.FORWARD);
        rightscooperservo.setDirection(Servo.Direction.FORWARD);
//        bootkickerservo.setDirection(Servo.Direction.REVERSE);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)l
//
//        switch (BootStateVar) {
//            case UP:
//                setpose(bootkickerservo,SERVOUTIL.bootkickerup);
//                break;
//            case DOWN:
//                setpose(bootkickerservo,SERVOUTIL.bootkickerdown);
//                break;
//            case IDLE:
//
//                break;
//        }
        switch (RightScooperStateVar){
            case UP:
                setpose(rightscooperservo, SERVOUTIL.rightscooperup);
                break;
            case DOWN:
                setpose(rightscooperservo, SERVOUTIL.rightscooperdown);
                break;
            case IDLE:

                break;
        }
        switch (LeftScooperStateVar){
            case UP:
                setpose(leftscooperservo, SERVOUTIL.leftscooperup);
                break;
            case DOWN:
                setpose(leftscooperservo, SERVOUTIL.leftscooperdown);
                break;
            case IDLE:

                break;
        }
        switch (LeftELEStateVar) {
            case UP:
                setpose(leftelevatorservo, SERVOUTIL.leftelevatorservoup);
                break;
            case DOWN:
                setpose(leftelevatorservo, SERVOUTIL.leftelevatorservodown);
                break;
            case IDLE:

                break;
        }
        switch (RightELEStateVar) {
                case DOWN:
                setpose(rightelevatorservo, SERVOUTIL.rightelevatorservoup);
                break;
            case UP:
                setpose(rightelevatorservo, SERVOUTIL.rightelevatorservodown);
                break;
            case IDLE:

                break;
        }
        switch (MiddleELEStateVar) {
            case DOWN:
                setpose(middleelevatorservo, SERVOUTIL.middleelevatorservoup);
                break;
            case UP:
                setpose(middleelevatorservo, SERVOUTIL.middleelevatorservodown);
                break;
            case SLIGHT:
                setpose(middleelevatorservo,SERVOUTIL.middleelevatorservoupslight);
                break;
            case IDLE:

                break;
        }


    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        // add telemetry data here

    }

    public Action ServoAction(SERVOSUB servosub, List<Runnable> funcs) {
        return new ServoAction(servosub,funcs);
    }

    class ServoAction implements Action {
        List<Runnable> funcs;
        private SERVOSUB servosub;

        public ServoAction(SERVOSUB servosub, List<Runnable> funcs) {
            this.funcs = funcs;
            this.servosub = servosub;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            servosub.update();// removes the need for the update to be run after simply updating a claw

            return false;
        }
    }
    public Action servoAction(List<Runnable> funcs){
        return new ServoAction(this, funcs);
    }
}