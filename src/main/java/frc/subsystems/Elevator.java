// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Elevator extends SubsystemBase{
    double speed = 0.4;
    private CANSparkMax elevatorMotor;

    public Elevator(){
        elevatorMotor = new CANSparkMax(RobotMap.INDEXER_MOTOR, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(40);
        elevatorMotor.burnFlash();
    }
    public void moveToTop(){
        elevatorMotor.set(speed);
    }
    public void moveToBottom(){
        elevatorMotor.set(-speed);
    }
    public void stop(){
        elevatorMotor.set(0.0);
    }
    
}