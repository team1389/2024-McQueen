package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class Indexer extends SubsystemBase{
    double speed = 0.4;
    private CANSparkMax indexerMotor;

    public Indexer(){
        indexerMotor = new CANSparkMax(RobotMap.INDEXER_MOTOR, MotorType.kBrushless);
        indexerMotor.setSmartCurrentLimit(40);
        indexerMotor.burnFlash();
    }

    public void moveToShoot() {
        indexerMotor.set(speed);
    }

    public void moveToAmp(){
        indexerMotor.set(-speed);
    }

    public void stop(){
        indexerMotor.set(0);
    }

}
