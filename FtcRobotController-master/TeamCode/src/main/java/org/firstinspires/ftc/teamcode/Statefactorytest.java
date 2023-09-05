package org.firstinspires.ftc.teamcode;

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
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();


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

        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );


        waitForStart(); //#######################START BUTTON PRESSED###########################

        stateMachine.start();
        imu.resetYaw();
        long lastTime=0;

        while (opModeIsActive()){
            stateMachine.update();
            telemetry.addData("Runtime", runtime.milliseconds());
            double looptime=1000000000/(System.nanoTime()-lastTime);
            lastTime=System.nanoTime();
            telemetry.addData("Looptime", looptime);
            telemetry.update();
        }

    }
}
