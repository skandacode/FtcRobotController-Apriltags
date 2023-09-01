package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Statefactorytest extends LinearOpMode {
    enum States {
        READY,
        EXTENDINTAKE,
        GRAB,
        RETRACTINTAKE,
        TRANSFERCONE,
        EXTENDOUTTAKE,
        FLIPDEPOSIT,
        RETRACTOUTTAKE
    }
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        drive=new MecanumDrive(
                new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312)
        );

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.READY)
                .onEnter(()->{telemetry.addLine("Ready state"); runtime.reset();})
                .onExit(()->telemetry.addLine("End ready state"))
                .transition(()->gamepad1.a)

                .state(States.EXTENDINTAKE)
                .onEnter(()->{telemetry.addLine("Extend state"); runtime.reset();})
                .onExit(()->telemetry.addLine("End extend state"))
                .transition(()->runtime.milliseconds()>10000, States.READY)

                .build();


        GamepadEx driverOp = new GamepadEx(gamepad1);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );


        waitForStart(); //#######################START BUTTON PRESSED###########################

        stateMachine.start();

        while (opModeIsActive()){
            drive.driveFieldCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX(),
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), false);
            stateMachine.update();
            telemetry.addData("Runtime", runtime.milliseconds());
            telemetry.update();
        }

    }
}
