package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.UTILITIES.FlyUTIL;


@TeleOp(name = "flytune")
public class flytune extends LinearOpMode {
    public DcMotorEx gunmotorL;
    public DcMotorEx gunmotorR;

    public enum Velocity {High, Low}

    private Velocity VelocityStateVar = Velocity.High;
    double currenttargetvelo = FlyUTIL.highvelo;


    @Override
    public void runOpMode() {
        gunmotorL = hardwareMap.get(DcMotorEx.class, "gunmotorL");
        gunmotorR = hardwareMap.get(DcMotorEx.class, "gunmotorR");
        gunmotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        gunmotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gunmotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gunmotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
        gunmotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        gunmotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();
        while (opModeIsActive()) {
            if (VelocityStateVar == Velocity.High && gamepad2.crossWasPressed()) {
                VelocityStateVar = Velocity.Low;
                currenttargetvelo = FlyUTIL.lowvelo;
            } else if (VelocityStateVar == Velocity.Low && gamepad2.crossWasPressed()) {
                VelocityStateVar = Velocity.High;
                currenttargetvelo = FlyUTIL.highvelo;
            }

            PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(FlyUTIL.p,0,0,FlyUTIL.f);
            gunmotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients2);
            gunmotorL.setVelocity(currenttargetvelo);
            gunmotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients2);
            gunmotorR.setVelocity(currenttargetvelo);

            double currentVelo = gunmotorL.getVelocity();
            telemetry.addData("targetvelo",currenttargetvelo);
            telemetry.addData("currentVelo",currentVelo);
            telemetry.addData("current Velo",currentVelo);
            telemetry.update();
        }
    }
}
