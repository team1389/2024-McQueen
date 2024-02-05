package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
    //for other stuff later
    // armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
public class Arm extends SubsystemBase{
    //radians per second?
    private final double rotateSpeed = 1;
    private CANSparkMax armMotor;
    private SparkMaxAlternateEncoder enconder; 

    public Arm(){
    armMotor = new CANSparkMax(RobotMap.ARM_MOTOR,MotorType.kBrushless);
    armMotor.setSmartCurrentLimit(40);
    armMotor.burnFlash();
    //idk what the last paramater is, 0 for placeholder
    //I have no diea what to do for this
    // enconder = armMotor.getAlternateEncoder(, 0);
    }

    public double getElbowPos() {
        double pos = enconder.getPosition();
        return (pos < 0.5) ? pos + (2*Math.PI) : pos;
    }
    public void setAngle(){
        armMotor.set(rotateSpeed);
    }
    public void stop(){
        armMotor.set(0);
    }
}
