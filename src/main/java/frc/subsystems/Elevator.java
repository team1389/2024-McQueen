// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Elevator extends SubsystemBase{
    double speed = .5;
    double low = 0;
    double high = 0;
    private CANSparkFlex elevatorMotor;
    private DutyCycleEncoder elevatorAbsEncoder;

    public Elevator(){
        elevatorMotor = new CANSparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.burnFlash();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        //correct port
        elevatorAbsEncoder = new DutyCycleEncoder(7);
    }
    public void moveElevator(double power){
        elevatorMotor.set(power);
    }
    public double getAbsElevatorPosition(){
        return elevatorAbsEncoder.getAbsolutePosition();
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
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder ABS Position", getAbsElevatorPosition()); 
    }
}
