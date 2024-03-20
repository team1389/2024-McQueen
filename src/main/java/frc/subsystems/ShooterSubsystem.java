package frc.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import javax.swing.text.StyleContext.SmallAttributeSet;

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
import frc.robot.RobotMap.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private double shootSpeed = 1; // percent of max motor speed -1 to 1
    private final double wristSpeed = .15; // percent of max motor speed
    private CANSparkFlex shootBottomController;
    private CANSparkFlex shootTopController;
    private RelativeEncoder shootEncoderBottom;
    private RelativeEncoder shootEncoderTop;
    private CANSparkFlex wristController;
    public boolean controllerInterrupt = true;
    public double wristTarget;
    public double shootTarget;
    public double setpoint = 3000;
    private DutyCycleEncoder wristAbsEncoder;

    // private final ProfiledPIDController pidWristController = new ProfiledPIDController(.5, 0, 0, backConstraints);
    private final PIDController pidWristController; //maybe sparkPidController

    private RelativeEncoder wristEncoder; // thru bore encoder
    private final SparkPIDController bottomPidController;
    private final SparkPIDController topPidController;

    public ShooterSubsystem(){
        SmartDashboard.putBoolean("Is setting wrist", false);
        shootBottomController = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_BOTTOM, MotorType.kBrushless);
        shootTopController = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_TOP, MotorType.kBrushless);
        wristController = new CANSparkFlex(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);

        shootEncoderBottom = shootBottomController.getEncoder(); //gets the build-in motor encoder
        shootEncoderTop = shootTopController.getEncoder(); //gets the build-in motor encoder

        shootBottomController.setSmartCurrentLimit(40); // neo vortex specifications, 40 amp breaker, cannot exceed 40
        shootBottomController.setInverted(true);
        shootBottomController.setIdleMode(IdleMode.kCoast);

        bottomPidController = shootBottomController.getPIDController();
        bottomPidController.setFeedbackDevice(shootEncoderBottom);

        bottomPidController.setP(ShooterConstants.kBottom_P);
        bottomPidController.setI(ShooterConstants.kBottom_I);
        bottomPidController.setD(ShooterConstants.kBottom_D);
        // bottomPidController.setIZone(0);
        bottomPidController.setFF(ShooterConstants.kBottom_FF);
        bottomPidController.setOutputRange(ShooterConstants.kBottom_MIN_OUTPUT, ShooterConstants.kBottom_MAX_OUTPUT); //min is reverse power minium, max is forward power maximum

        shootBottomController.burnFlash();
        shootTopController.setSmartCurrentLimit(40);
        shootTopController.setInverted(true);
        shootTopController.setIdleMode(IdleMode.kCoast); // neo vortex specifications
    
        topPidController = shootTopController.getPIDController();
        topPidController.setFeedbackDevice(shootEncoderTop);

        topPidController.setP(ShooterConstants.kTop_P);
        topPidController.setI(ShooterConstants.kTop_I);
        topPidController.setD(ShooterConstants.kTop_D);
        // topPidController.setIZone(0);
        topPidController.setFF(ShooterConstants.kTop_FF);
        topPidController.setOutputRange(ShooterConstants.kTop_MIN_OUTPUT,ShooterConstants.kTop_MAX_OUTPUT); //min is reverse power minium, max is forward power maximum

        shootTopController.burnFlash();
        wristController.setSmartCurrentLimit(40); // neo vortex specifications
        wristController.setIdleMode(IdleMode.kBrake);
        wristController.burnFlash();
        wristEncoder = wristController.getEncoder(); //through bore encoder
        
        // wristPidController.setP(ModuleConstants.P_TURNING);
        // wristPidController.setI(ModuleConstants.I_TURNING);
        // wristPidController.setD(ModuleConstants.D_TURNING);

        // SmartDashboard.putNumber("Turning P", ModuleConstants.P_WRIST);
        // SmartDashboard.putNumber("Turning I", ModuleConstants.I_WRIST);
        // SmartDashboard.putNumber("Turning D", ModuleConstants.D_WRIST);
        
        pidWristController = new PIDController(ShooterConstants.kWrist_P, ShooterConstants.kWrist_I, ShooterConstants.kWrist_D);

        pidWristController.setP(ShooterConstants.kWrist_P);
        pidWristController.setI(ShooterConstants.kWrist_I);
        pidWristController.setD(ShooterConstants.kWrist_D);

        //Uncomment the below code to tune pid values in smartdashboard - you also need to uncommet the part in periodic
        // SmartDashboard.putNumber("P Wrist", 0.055);
        // SmartDashboard.putNumber("I Wrist", 0.01);
        // SmartDashboard.putNumber("D Wrist", 0.000);
        // SmartDashboard.putNumber("Wrist Motor Speed", 0.25);

        // SmartDashboard.putNumber("P Bottom Shooter", 0.1);
        // SmartDashboard.putNumber("I Bottom Shooter", 0.0);
        // SmartDashboard.putNumber("D Bottom Shooter", 0.000);
        SmartDashboard.putNumber("FF Bottom Shooter", 1.75);

        // SmartDashboard.putNumber("P Top Shooter", 0.1);
        // SmartDashboard.putNumber("I Top Shooter", 0.0);
        // SmartDashboard.putNumber("D Top Shooter", 0.000);
        SmartDashboard.putNumber("FF Top Shooter", 1.75);

        //correct value
        wristAbsEncoder = new DutyCycleEncoder(8); // this is a through bore encoder
    }

    public void setWrist(double desiredAngle){
        SmartDashboard.putBoolean("Is setting wrist", true);
        desiredAngle = MathUtil.clamp(desiredAngle, ShooterConstants.kMinWristAngle, ShooterConstants.kMaxWristAngle);
        //needs to be multiplied by 100 otherwise the pidloop cancels as the difference between two value is quite small
        double wristPower = pidWristController.calculate(getAbsWristPosition()*100, desiredAngle*100);
        moveWrist(wristPower);   
    }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.3, 0.3);
        wristController.set(power);
    }

    public void runShoot(double desiredRPM) {
        setpoint = desiredRPM;
        topPidController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
        bottomPidController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
    }

    public void runShootSpeed(){
        shootBottomController.set(shootSpeed);
        shootTopController.set(shootSpeed);
    }
    
    public void runWristUp(){
        wristController.set(wristSpeed);
    }

    public boolean isAtTargetRPM(double setpoint){
        this.setpoint = setpoint;
        return getTopSpeedRPM() > (setpoint - 500);
        // return ((getTopSpeedRPM()-setpoint)<50)&&((getBottomSpeedRPM()-setpoint)<50);
    }

    public void runWristDown(){
        wristController.set(-.05);
    } 

    public void stopWrist(){
        wristController.set(0);
    }

    public void stop(){
        shootBottomController.set(0);
        shootTopController.set(0);
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

    // public double getRPM(){
    //     return wristAbsEncoder.getFrequency() * 60;
    // }

    // public void setSpeed(){
    //     shootSpeed = 3.6 * (getRPM()/5252);
    // }

    public double getTopSpeedRPM(){
        return shootEncoderTop.getVelocity();
    }

    public double getBottomSpeedRPM(){
        return shootEncoderBottom.getVelocity();
    }

    public void holdPosition(){
        // calculated line of best fit from tested points
        if(getAbsWristPosition()<0.95 && getAbsWristPosition()>0.8){
            moveWrist(-0.132*getAbsWristPosition() + 0.133);
            // moveWrist(-0.075*getAbsWristPosition() + 0.0914);
        }
        else {
            stopWrist();
        }
        // SmartDashboard.putBoolean("Inside hold position", true);
    }

    
    @Override
    public void periodic(){

        SmartDashboard.putNumber("wristController Encoder ABS Position", getAbsWristPosition()); 

        SmartDashboard.putBoolean("Is at Target RPM", isAtTargetRPM(setpoint));

        //Uncomment the below code to tune pid values in smartdashboard - you also need to uncommet the part in periodic
        // pidWristController.setP(SmartDashboard.getNumber("P Wrist", .055));
        // pidWristController.setI(SmartDashboard.getNumber("I Wrist", 0.01));
        // pidWristController.setD(SmartDashboard.getNumber("D Wrist", 0.000));

        // bottomPidController.setP(SmartDashboard.getNumber("P Bottom Shooter", 0.1));
        // bottomPidController.setI(SmartDashboard.getNumber("I Bottom Shooter", 0));
        // bottomPidController.setD(SmartDashboard.getNumber("D Bottom Shooter", 0.000));
            bottomPidController.setFF(SmartDashboard.getNumber("FF Bottom Shooter", 0.000));

        // topPidController.setP(SmartDashboard.getNumber("P Top Shooter", 0.1));
        // topPidController.setI(SmartDashboard.getNumber("I Top Shooter", 0));
        // topPidController.setD(SmartDashboard.getNumber("D Top Shooter", 0.000));
            topPidController.setFF(SmartDashboard.getNumber("FF Top Shooter", 0.000));

        SmartDashboard.putNumber("Shoot Bottom Encoder CPR", shootEncoderBottom.getCountsPerRevolution());
        SmartDashboard.putNumber("Shoot Bottom RPM", getBottomSpeedRPM());
        SmartDashboard.putNumber("Shoot Bottom Encoder Position", shootEncoderBottom.getPosition());

        SmartDashboard.putNumber("Shoot Top Encoder CPR", shootEncoderTop.getCountsPerRevolution());
        SmartDashboard.putNumber("Shoot Top RPM", getTopSpeedRPM());
        SmartDashboard.putNumber("Shoot Top Encoder Position", shootEncoderTop.getPosition());
    }
}