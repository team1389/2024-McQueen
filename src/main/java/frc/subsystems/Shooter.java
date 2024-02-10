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
    private CANSparkMax shootLeft;
    private CANSparkMax shootRight;
    private CANSparkMax wrist;
    public boolean controllerInterrupt = false;
    private PIDController pidWrist;
    public double wristTarget;

    private AbsoluteEncoder wristEncoder;
// two motors the spin opposite for the both sets of shooter wheels
    public Shooter(){
        shootLeft = new CANSparkMax(RobotMap.SHOOT_LEFT, MotorType.kBrushless);
        shootRight = new CANSparkMax(RobotMap.SHOOT_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        shootLeft.setSmartCurrentLimit(40);
        shootLeft.burnFlash();
        shootRight.setSmartCurrentLimit(40);
        shootRight.burnFlash();
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
        shootLeft.set(shootSpeed);
        shootRight.set(-shootSpeed); //inverse the direction in rev
    }

    public void stop(){
        shootLeft.set(0);
        shootRight.set(0);
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
