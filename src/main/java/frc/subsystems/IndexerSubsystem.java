package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;

public class IndexerSubsystem extends SubsystemBase{
    double indexerMotorSpeed = .2; //.2
    private CANSparkFlex indexerMotor;

    public IndexerSubsystem(){
        indexerMotor = new CANSparkFlex(RobotMap.MotorPorts.INDEXER_MOTOR, MotorType.kBrushless);
        indexerMotor.setSmartCurrentLimit(40);
        indexerMotor.burnFlash();
    }

    public void moveToShoot() {
        indexerMotor.set(indexerMotorSpeed);
    }

    public void moveToAmp(){
        indexerMotor.set(-indexerMotorSpeed);
    }

    public void stop(){
        indexerMotor.set(0);
    }

}
