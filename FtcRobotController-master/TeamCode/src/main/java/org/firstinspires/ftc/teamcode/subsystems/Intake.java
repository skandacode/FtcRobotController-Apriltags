package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake implements Subsystem{

    Motor intakeMotor;
    Servo clawServo, flipServo1, flipServo2;
    ColorSensor clawSensor;
    PIDFController intakePID;


    @Override
    public void init(HardwareMap hardwareMap) {
        intakeMotor=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intakeMotor.resetEncoder();

        clawServo= hardwareMap.servo.get("claw");
        flipServo1= hardwareMap.servo.get("flip1");
        flipServo2= hardwareMap.servo.get("flip2");

        flipServo1.setDirection(Servo.Direction.FORWARD);
        flipServo2.setDirection(Servo.Direction.REVERSE);

        clawSensor = hardwareMap.get(ColorSensor.class, "sensor");

        intakePID = new PIDFController(0.005, 0, 0, 0);
        intakePID.setSetPoint(0);
    }
    public void setIntakePosition(int position){
        intakePID.setSetPoint(position);
    }
    @Override
    public void update() {
        intakeMotor.set(intakePID.calculate(intakeMotor.getCurrentPosition()));
    }
}
