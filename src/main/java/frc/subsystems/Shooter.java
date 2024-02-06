package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{
    private final double shootSpeed = 1;
    private CANSparkMax shootTop;
    private CANSparkMax shootBottom;
    private CANSparkMax wrist;
    public boolean controllerInterrupt = false;
    private PIDController pidWrist;
    public double wristTarget;

    private AbsoluteEncoder wristEncoder;

 


    public Shooter(){
        shootTop = new CANSparkMax(RobotMap.SHOOT_TOP, MotorType.kBrushless);
        shootBottom = new CANSparkMax(RobotMap.SHOOT_BOTTOM, MotorType.kBrushless);
        wrist = new CANSparkMax(RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        shootTop.setSmartCurrentLimit(40);
        shootTop.burnFlash();
        shootBottom.setSmartCurrentLimit(40);
        shootBottom.burnFlash();
        wrist.setSmartCurrentLimit(40);
        wrist.burnFlash();

        //decide pid values later, P, I, D
        pidWrist = new PIDController(0.5, 0, 0);
    }

    public double setWrist(double pos) {
        var temp = wristTarget;
        wristTarget = pos;
        SmartDashboard.putNumber("Wrist target", wristTarget);
        return temp;
    }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.3, 0.3);
        wrist.set(power);
    }

    public void runShoot() {
        shootTop.set(shootSpeed);
        shootBottom.set(-shootSpeed);
    }

    public void stop(){
        shootTop.set(0);
        shootBottom.set(0);
    }

    public double getWristPos() {
        return wristEncoder.getPosition();
    }

    // public void resetEnconders(){
    //     wristEncoder.setPosition(0);
    // }
    
    @Override
    public void periodic(){
        wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());
        double wristPower = 0;
        if (!controllerInterrupt) {      
            wristPower = pidWrist.calculate(getWristPos(), wristTarget);
            moveWrist(wristPower);       
        }
    }
}
