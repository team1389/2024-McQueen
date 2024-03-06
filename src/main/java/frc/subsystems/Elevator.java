// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Elevator extends SubsystemBase{
    double speed = 1;
    public double pos = .5;
    private CANSparkFlex elevatorMotor;
    private DutyCycleEncoder elevatorEncoder;
    private final PIDController elevatorPid;
    public boolean controllerInterrupt = true;


    public Elevator(){
        elevatorMotor = new CANSparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.burnFlash();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorEncoder = new DutyCycleEncoder(RobotMap.MotorPorts.ELEVATOR_ENCODER);
        elevatorPid = new PIDController(0, 0, 0);

        SmartDashboard.putNumber("P Elevator", 0.12);
        SmartDashboard.putNumber("I Elevator", 0.005);
        SmartDashboard.putNumber("D Elevator", 0.000);
        
    }
    public void moveElevator(double power){
        elevatorMotor.set(power);
    }
    public double getAbsElevatorPosition(){
        return elevatorEncoder.getAbsolutePosition();
    }
    public void moveToTop(){
        elevatorMotor.set(-speed);
    }
    public void moveToBottom(){
        elevatorMotor.set(speed);
    }
    public void stop(){
        elevatorMotor.set(0.0);
    }

    public void setSetpoint(double setpoint){
        pos = setpoint;
    }

    public void setElevator(double pos){
        pos = MathUtil.clamp(pos, .511, .605); 
        elevatorMotor.set(elevatorPid.calculate(elevatorEncoder.getDistance(), pos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Abs Pos", elevatorEncoder.getAbsolutePosition());

        elevatorPid.setP(SmartDashboard.getNumber("P Elevator", .12));
        elevatorPid.setI(SmartDashboard.getNumber("I Elevator", 0.005));
        elevatorPid.setD(SmartDashboard.getNumber("D Elevator", 0.000));
        
    }
}
