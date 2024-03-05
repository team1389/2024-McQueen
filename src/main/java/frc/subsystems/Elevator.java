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
    public double setpoint = .5;
    private CANSparkFlex elevatorMotor;
    private DutyCycleEncoder elevatorEncoder;
    private final PIDController elevatorPid;

    public Elevator(){
        elevatorMotor = new CANSparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.burnFlash();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorEncoder = new DutyCycleEncoder(RobotMap.MotorPorts.ELEVATOR_ENCODER);
        elevatorPid = new PIDController(0, 0, 0);
        
    }
    public void moveElevator(double power){
        elevatorMotor.set(power);
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

    public void setSetpoint(double pos){
        setpoint = pos;
    }

    public void setElevator(double pos){
        pos = MathUtil.clamp(pos, 0, 1); // change
        elevatorMotor.set(elevatorPid.calculate(elevatorEncoder.getDistance(), setpoint));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Abs Pos", elevatorEncoder.getAbsolutePosition());
        
    }
}
