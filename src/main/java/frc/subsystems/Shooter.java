package frc.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{
    private final double shootSpeed = 1; // percent of max motor speed
    private final double wristSpeed = .15; // percent of max motor speed
    private CANSparkFlex shootLeft;
    private CANSparkFlex shootRight;
    private CANSparkFlex wrist;
    private final SparkPIDController wristPidController;
    public boolean controllerInterrupt = true;
    private final double maxWristEncoderVal = 0.962;
    private final double minWristEncoderVal = 0.801;
 //   private PIDController pidWrist;
    public double wristTarget;
    public double madyannPos;


    private final TrapezoidProfile.Constraints backConstraints = new TrapezoidProfile.Constraints(10, 20);
    // private final ProfiledPIDController pidWrist = new ProfiledPIDController(.5, 0, 0, backConstraints);
    private final PIDController pidWrist; //maybe sparkPidController
    

    private DutyCycleEncoder wristEncoder; // thru bore encoder


    public Shooter(){
        shootLeft = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_LEFT, MotorType.kBrushless);
        shootRight = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkFlex(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);
        shootLeft.setSmartCurrentLimit(40); // neo vortex specifications, 40 amp breaker, cannot exceed 40
        shootLeft.burnFlash();
        shootRight.setSmartCurrentLimit(40); // neo vortex specifications
        shootRight.burnFlash();
        wrist.setSmartCurrentLimit(40); // neo vortex specifications
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.burnFlash();
        //trial and error
        wristEncoder = new DutyCycleEncoder(8); //through bore encoder
        wristPidController = wrist.getPIDController();
        // wristPidController.setP(ModuleConstants.P_TURNING);
        // wristPidController.setI(ModuleConstants.I_TURNING);
        // wristPidController.setD(ModuleConstants.D_TURNING);

        // SmartDashboard.putNumber("Turning P", ModuleConstants.P_WRIST);
        // SmartDashboard.putNumber("Turning I", ModuleConstants.I_WRIST);
        // SmartDashboard.putNumber("Turning D", ModuleConstants.D_WRIST);

        
        

        wristPidController.setOutputRange(-1, 1);

        
        //decide pid values later, P, I, D
        pidWrist = new PIDController(0.25, 0, 0);
    //   pidWrist = wrist.getPIDController();

    //   pidWrist.setFeedbackDevice(wristEncoder);

        pidWrist.setP(.25);
        pidWrist.setI(0);
        pidWrist.setD(0);

        madyannPos = 0.85;
        SmartDashboard.putNumber("P Wrist", 0.25);
        SmartDashboard.putNumber("I Wrist", 0.0000);
        SmartDashboard.putNumber("D Wrist", 0.000);
        SmartDashboard.putNumber("Madyanns Funny Number", .85);
        SmartDashboard.putNumber("Target Angle", 0.5);
    }
    
    public double setWrist(double pos) {
        SmartDashboard.putNumber("Wrist target", pos);
        wristPidController.setReference(pos, CANSparkMax.ControlType.kPosition);
        return pos;
    }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.05, 0.1);
        wrist.set(power);
    }

    public double getMadyannsNumber(){
        return madyannPos;
    }

    public void runShoot() {
        shootLeft.set(shootSpeed);
        shootRight.set(-shootSpeed); //inverse the direction in rev
    }

    public void runWristUp(){
        wrist.set(-wristSpeed);
    }

    public void runWristDown(){
        wrist.set(.05);
    } 

    public void stopWrist(){
        wrist.set(-.01);
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

    public double getWristAngle(){
        return ((getWristPos()-minWristEncoderVal) / (maxWristEncoderVal-minWristEncoderVal)) * 90;
    }

    public void resetWristPos() {
        wristEncoder.reset();
    }

    public void holdPosition(){
        wrist.set(pidWrist.calculate(getWristPos(), getTargetAngle()));
        SmartDashboard.putBoolean("Inside hold position", true);
    }

    public void setTargetAngle(double pos){
        wristTarget = pos;
    }

    public double getTargetAngle(){
        return wristTarget;
    }

    public void toggleControllerInterrupt(){
        controllerInterrupt = !controllerInterrupt;
    }
    
    @Override
    public void periodic(){
        // moveWrist(getWristPos());
        double wristAngle = (Math.PI / 2) - ((Math.PI * 2) - getWristPos() + Math.toRadians(25));
        SmartDashboard.putNumber("Wrist Encoder Distance", getWristDis());
        SmartDashboard.putNumber("encoder value in rotations", wristEncoder.get());

        SmartDashboard.putNumber("Wrist Encoder Absolute Position", wristEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Encoder Position", wristEncoder.getAbsolutePosition() - wristEncoder.getPositionOffset());
        SmartDashboard.putNumber("Wrist Angle", getWristAngle());
        SmartDashboard.putNumber("wristPos", getWristPos());
        // SmartDashboard.putNumber("P Wrist", 0.25);
        // SmartDashboard.putNumber("I Wrist", 0.0000);
        // SmartDashboard.putNumber("D Wrist", 0.000);
        // SmartDashboard.putNumber("Wrist Target", getTargetAngle());
     //   wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());
      //  double wristPower = 0;
      // wrist.set(pid.calculate(wristEncoder.getDistance(), getWristPos()));
      if(!controllerInterrupt){
        // setWrist(.5);
        // setWrist(SmartDashboard.getNumber("Madyanns Funny Number",.85));
        wrist.set(pidWrist.calculate(getWristPos(), getTargetAngle()));
        
    //     double wristPower = 0; test this too

    //    wristPower = pidWrist.calculate(getWristPos(), wristTarget);
    //     moveWrist(wristPower);

        setTargetAngle(SmartDashboard.getNumber("Target Angle", getTargetAngle()));
        pidWrist.setP(SmartDashboard.getNumber("P Wrist", 0.25));
        pidWrist.setI(SmartDashboard.getNumber("I Wrist", 0.0000));
        pidWrist.setD(SmartDashboard.getNumber("D Wrist", 0.000));
          //  moveWrist(wristPower);
     }
    }
}
