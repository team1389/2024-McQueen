package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{
    private final double shootSpeed = 1;
    private CANSparkMax shootTop;
    private CANSparkMax shootBottom;
 


    public Shooter(){
        shootTop = new CANSparkMax(RobotMap.SHOOT_TOP, MotorType.kBrushless);
        shootBottom = new CANSparkMax(RobotMap.SHOOT_BOTTOM, MotorType.kBrushless);
        shootTop.setSmartCurrentLimit(40);
        shootTop.burnFlash();
        shootBottom.setSmartCurrentLimit(40);
        shootBottom.burnFlash();
    }

    public void runShoot() {
        shootTop.set(shootSpeed);
        shootBottom.set(-shootSpeed);
    }

    public void stop(){
        shootTop.set(0);
        shootBottom.set(0);
    }
}
