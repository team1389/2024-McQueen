package frc.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ModuleConstants;

public class Shooter extends SubsystemBase{
    private final double shootSpeed = -.3;
    private final double wristSpeed = .15;
    private CANSparkFlex shootLeft;
    private CANSparkFlex shootRight;
    private CANSparkMax wrist;
    public boolean controllerInterrupt = false;
 //   private PIDController pidWrist;
    public double wristTarget;
    private PIDController pid = new PIDController(.5, 0, 0);


    private final TrapezoidProfile.Constraints backConstraints = new TrapezoidProfile.Constraints(10, 20);
    private final ProfiledPIDController pidWrist = new ProfiledPIDController(.5, 0, 0, backConstraints);
    

    private DutyCycleEncoder wristEncoder;


    public Shooter(){
        shootLeft = new CANSparkFlex(RobotMap.SHOOT_LEFT, MotorType.kBrushless);
        shootRight = new CANSparkFlex(RobotMap.SHOOT_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkMax(RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        shootLeft.setSmartCurrentLimit(40);
        shootLeft.burnFlash();
        shootRight.setSmartCurrentLimit(40);
        shootRight.burnFlash();
        wrist.setSmartCurrentLimit(40);
        wrist.burnFlash();
        wrist.setIdleMode(IdleMode.kBrake);
        //trial and error
        wristEncoder = new DutyCycleEncoder(8);
    //    wristEncoder = new AbsoluteEncoder(RobotMap.) 

        //decide pid values later, P, I, D
    //    pidWrist = new PIDController(0.5, 0, 0);
    }

    public double setWrist(double pos) {
        var temp = wristTarget;
        wristTarget = pos;
        SmartDashboard.putNumber("Wrist target", wristTarget);
        return temp;
    }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.05, 0.1);
        wrist.set(power);
    }

    public void runShoot() {
        shootLeft.set(shootSpeed);
        shootRight.set(shootSpeed); //inverse the direction in rev
    }

    public void runWristUp(){
        wrist.set(-wristSpeed);
    }

    public void runWristDown(){
        wrist.set(.05);
    } 

    public void stopWrist(){
        wrist.set(0);
    }

    public void stop(){
        shootLeft.set(0);
        shootRight.set(0);
    }

    public double getWristDis() {
        return wristEncoder.getDistance();
    }

    public double getWristPos(){
        return wristEncoder.getAbsolutePosition() - wristEncoder.getPositionOffset();
    }

    public void resetWristPos() {
        wristEncoder.reset();
    }
    double wristPower = 0;
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Encoder Distance", getWristDis());
        SmartDashboard.putNumber("encoder value in rotations", wristEncoder.get());
        SmartDashboard.putNumber("Wrist Encoder Absolute Position", wristEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Encoder Position", wristEncoder.getAbsolutePosition() - wristEncoder.getPositionOffset());
     //   wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());
      //  double wristPower = 0;
      if(!controllerInterrupt){
            wristPower = pidWrist.calculate(getWristPos(), wristTarget);
            moveWrist(wristPower);
      }
    }
}
