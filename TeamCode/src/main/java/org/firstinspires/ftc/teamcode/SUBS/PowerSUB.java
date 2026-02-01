package org.firstinspires.ftc.teamcode.SUBS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UTILITIES.FlyUTIL;

import java.util.List;

public class PowerSUB {
    private DcMotor intakeR;
    private DcMotor intakeL;
    private DcMotorEx gunmotorR;
    private DcMotorEx gunmotorL;

    public enum gunSTATE {ON, OFF, REVERSE, MATCH,LEFTSHOT,RIGHTSHOT,MIDSHOT, IDLE}
//!help
    private PowerSUB.gunSTATE gunStateVar = PowerSUB.gunSTATE.IDLE;

    public void gunon() {
        gunStateVar = gunSTATE.ON;
    }

    public void gunoff() {
        gunStateVar = gunSTATE.OFF;
    }
    public void gunreverse(){gunStateVar = gunSTATE.REVERSE;}
    public void gunmatch(){gunStateVar = gunSTATE.MATCH;}
    public void gunidle(){gunStateVar = gunSTATE.IDLE;}
    public void gunleftshot(){gunStateVar = gunSTATE.LEFTSHOT;}
    public void gunrightshot(){gunStateVar = gunSTATE.RIGHTSHOT;}
    public void gunmidshot(){gunStateVar = gunSTATE.MIDSHOT;}

    public enum intakeSTATE {ON, OFF, REVERSE, IDLE}

    private PowerSUB.intakeSTATE intakeStateVar = PowerSUB.intakeSTATE.IDLE;

    public void intakeon() {
        intakeStateVar = intakeSTATE.ON;
    }

    public void intakeoff() {
        intakeStateVar = intakeSTATE.OFF;
    }
    public void intakereverse(){intakeStateVar = intakeSTATE.REVERSE;}



    //this is where you put all enums and variables
    public PowerSUB(HardwareMap hwMap) {
        gunmotorL = hwMap.get(DcMotorEx.class,"gunmotorL");
        gunmotorR = hwMap.get(DcMotorEx.class,"gunmotorR");
        intakeR = hwMap.get(DcMotor.class,"intakeR");
        intakeL = hwMap.get(DcMotor.class, "intakeL");
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        gunmotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        gunmotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
        gunmotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        gunmotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)
        switch (intakeStateVar) {
            case ON:
                intakeR.setPower(1);
                intakeL.setPower(1);
                break;
            case OFF:
                intakeR.setPower(0);
                intakeL.setPower(0);
                break;
            case REVERSE:
                intakeR.setPower(-0.5);
                intakeL.setPower(-0.5);
                break;
            case IDLE:

                break;
        }

        switch (gunStateVar) {
            case ON:
                gunmotorR.setVelocity(0.9*FlyUTIL.highvelo);

                gunmotorL.setVelocity(0.9*FlyUTIL.highvelo);
                break;
            case OFF:
                gunmotorL.setVelocity(0.7);

                gunmotorR.setVelocity(0.7);
                break;
            case MIDSHOT:
                gunmotorR.setVelocity(0.8);
                gunmotorL.setVelocity(0.8);
            case RIGHTSHOT:
                gunmotorL.setVelocity(0.9);

                gunmotorR.setVelocity(0.9);
            case LEFTSHOT:
                gunmotorR.setVelocity(0);

                gunmotorL.setVelocity(0);

            case REVERSE:
                gunmotorL.setVelocity(1*FlyUTIL.highvelo);

                gunmotorR.setVelocity(1*FlyUTIL.highvelo);
                break;
            case MATCH:
                gunmotorR.setVelocity(0.9*FlyUTIL.highvelo);

                gunmotorL.setVelocity(0.9*FlyUTIL.highvelo);
                break;
            case IDLE:
                gunmotorR.setVelocity(0.0*FlyUTIL.highvelo);

                gunmotorL.setVelocity(0.0*FlyUTIL.highvelo);
                break;
        }
    }

    // this is where you put your update functions to switch between states

    public void telemetry(Telemetry telemetry) {
        double currentVeloL = gunmotorL.getVelocity();
        double currentVeloR = gunmotorR.getVelocity();
        telemetry.addData("RVELO",currentVeloR);
        telemetry.addData("LVELO",currentVeloL);

    }
        // add telemetry data here


        class MotorAction implements Action {
            List<Runnable> funcs;
            private PowerSUB powersub;

            public MotorAction(PowerSUB powersub, List<Runnable> funcs) {
                this.funcs = funcs;
                this.powersub = powersub;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (Runnable func : funcs) {
                    func.run();
                }
                powersub.update();// removes the need for the update to be run after simply updating a claw

                return false;
            }

    }
    public Action gunAction(List<Runnable> funcs){
        return new MotorAction(this, funcs);
    }
}