package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class Rizzt extends SubsystemBase{
    double speed = 0.25;
    private CANSparkMax rizztMotor;

    public Rizzt(){
        rizztMotor = new CANSparkMax(RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        rizztMotor.setSmartCurrentLimit(40);
        rizztMotor.burnFlash();
    }

    public void moveUp() {
        rizztMotor.set(-speed);
    }

    public void moveDwon(){
        rizztMotor.set(speed);
    }

    public void stop(){
        rizztMotor.set(0);
    }

}
