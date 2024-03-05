package frc.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{
    private final double shootSpeed = -1; // percent of max motor speed
    private final double wristSpeed = .15; // percent of max motor speed
    private CANSparkFlex shootLeft;
    private CANSparkFlex shootRight;
    private RelativeEncoder shootEncoder;
    private CANSparkFlex wrist;
    private final SparkPIDController wristPidController;
    public boolean controllerInterrupt = true;
 //   private PIDController pidWrist;
    public double wristTarget;
    public double madyannPos;
    private DutyCycleEncoder wristAbsEncoder;

    // private final ProfiledPIDController pidWrist = new ProfiledPIDController(.5, 0, 0, backConstraints);
    private final PIDController pidWrist; //maybe sparkPidController
    

    private RelativeEncoder wristEncoder; // thru bore encoder


    public Shooter(){
        shootLeft = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_LEFT, MotorType.kBrushless);
        shootRight = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_RIGHT, MotorType.kBrushless);
        wrist = new CANSparkFlex(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);
       // shootRelativeEncoder = new RelativeEncoder();
        shootLeft.setSmartCurrentLimit(40); // neo vortex specifications, 40 amp breaker, cannot exceed 40
        shootLeft.burnFlash();
        shootRight.setSmartCurrentLimit(40); // neo vortex specifications
        shootRight.burnFlash();
        wrist.setSmartCurrentLimit(40); // neo vortex specifications
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.burnFlash();
        //trial and error
        wristEncoder = wrist.getEncoder(); //through bore encoder
        shootEncoder = shootLeft.getEncoder();
        
        wristPidController = wrist.getPIDController();
        // wristPidController.setP(ModuleConstants.P_TURNING);
        // wristPidController.setI(ModuleConstants.I_TURNING);
        // wristPidController.setD(ModuleConstants.D_TURNING);

        // SmartDashboard.putNumber("Turning P", ModuleConstants.P_WRIST);
        // SmartDashboard.putNumber("Turning I", ModuleConstants.I_WRIST);
        // SmartDashboard.putNumber("Turning D", ModuleConstants.D_WRIST);

        
        

        wristPidController.setOutputRange(-1, 1);

        
        //decide pid values later, P, I, D
        pidWrist = new PIDController(5, 0.1, 0);
    //   pidWrist = wrist.getPIDController();

    //   pidWrist.setFeedbackDevice(wristEncoder);

        pidWrist.setP(5);
        pidWrist.setI(.1);
        pidWrist.setD(0);

        madyannPos = 0.85;
        SmartDashboard.putNumber("P Wrist", 0.12);
        SmartDashboard.putNumber("I Wrist", 0.005);
        SmartDashboard.putNumber("D Wrist", 0.000);
        SmartDashboard.putNumber("Wrist Motor Speed", 0.25);
        wristAbsEncoder = new DutyCycleEncoder(8);
    }
    
    // public void setWrist(double angle) {
    //     wrist.set(pidWrist.calculate(wristEncoder.getPosition(), angle));
    // }

    public double setWristTarget(double pos) {
        var temp = wristTarget;
        wristTarget = pos;
        SmartDashboard.putNumber("Wrist target", wristTarget);
        return temp;
    }

    public void setWrist(double angle){
        angle = MathUtil.clamp(angle, 0.8, .95);
        //angle is from .8 to ~.96
        //set tolerance sets the error value to stop the pid loop at 
        // pidWrist.setTolerance(.005);
        double wristPower = pidWrist.calculate(getAbsWristPosition()*100, angle*100);
        moveWrist(wristPower);
    }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.3, 0.3);
        wrist.set(power);
    }

    public double getMadyannsNumber(){
        return madyannPos;
    }

    public void runShoot() {
        shootLeft.set(shootSpeed);
        shootRight.set(shootSpeed); //inversed the direction in rev
    }

    public void runWristUp(){
        wrist.set(wristSpeed);
    }

    public void runWristDown(){
        wrist.set(-.05);
    } 

    public void stopWrist(){
        wrist.set(0);
    }

    public void stop(){
        shootLeft.set(0);
        shootRight.set(0);
    }

    public void resetWristPos() {
        wristEncoder.setPosition(0);
    }

    public double getWristPosition(){
        return wristEncoder.getPosition();
    }

    public double getAbsWristPosition(){
        return wristAbsEncoder.getAbsolutePosition();
    }

    public void holdPosition(){
        // calculated line of best fit from tested points
        moveWrist(-0.1334*getAbsWristPosition() + 0.1343);
        // SmartDashboard.putBoolean("Inside hold position", true);
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

        SmartDashboard.putNumber("Wrist Encoder Position", getWristPosition()); 
        SmartDashboard.putNumber("Wrist Encoder ABS Position", getAbsWristPosition()); 

        pidWrist.setP(SmartDashboard.getNumber("P Wrist", .12));
        pidWrist.setI(SmartDashboard.getNumber("I Wrist", 0.005));
        pidWrist.setD(SmartDashboard.getNumber("D Wrist", 0.000));
        // SmartDashboard.putNumber("P Wrist", 0.25);
        // SmartDashboard.putNumber("I Wrist", 0.0000);
        // SmartDashboard.putNumber("D Wrist", 0.000);
        // SmartDashboard.putNumber("Wrist Target", getTargetAngle());
        //wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());
        // wrist.set(pid.calculate(wristEncoder.getDistance(), getWristPos()));
        // if(!controllerInterrupt){
        //     holdPosition();
        // }
        wristTarget = SmartDashboard.getNumber("Wrist target", getWristPosition());

        SmartDashboard.putNumber("Shoot Left Encoder CPR", shootEncoder.getCountsPerRevolution());
        SmartDashboard.putNumber("Shoot Left Encoder PPR", shootEncoder.getCountsPerRevolution()/4);
    }
}