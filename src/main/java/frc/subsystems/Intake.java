package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

//1 motor that spins the intake thingies

public class Intake extends SubsystemBase{
    private final double intakeSpeed = 1;
    private CANSparkMax intakeMotor;

    public Intake(){
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR,MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.burnFlash();
    }
    public void runIntake(){
        intakeMotor.set(intakeSpeed);
    }
    public void stop(){
        intakeMotor.set(0);
    }
}
