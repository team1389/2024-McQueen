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
import frc.robot.RobotMap.ShooterConstants;

public class Shooter extends SubsystemBase{
    private double shootSpeed = 1; // percent of max motor speed -1 to 1
    private final double wristSpeed = .15; // percent of max motor speed
    private CANSparkFlex shootBottomController;
    private CANSparkFlex shootTopController;
    private RelativeEncoder shootEncoderBottom;
    private RelativeEncoder shootEncoderTop;
    private CANSparkFlex wrist;
    public boolean controllerInterrupt = true;
 //   private PIDController pidWrist;
    public double wristTarget;
    public double shootTarget;
    public double madyannPos;
    private DutyCycleEncoder wristAbsEncoder;
    private final double MAX_DEGREES = 85;
    private final double MIN_DEGREES = 5;

    // private final ProfiledPIDController pidWrist = new ProfiledPIDController(.5, 0, 0, backConstraints);
    private final PIDController pidWrist; //maybe sparkPidController

    private RelativeEncoder wristEncoder; // thru bore encoder
    private final SparkPIDController bottomPidController;
    private final SparkPIDController topPidController;

    public Shooter(){
        shootBottomController = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_BOTTOM, MotorType.kBrushless);
        shootTopController = new CANSparkFlex(RobotMap.MotorPorts.SHOOT_TOP, MotorType.kBrushless);
        wrist = new CANSparkFlex(RobotMap.MotorPorts.WRIST_MOTOR, MotorType.kBrushless);

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
        bottomPidController.setFF(0.000175);
        bottomPidController.setOutputRange(0.3, 1); //min is reverse power minium, max is forward power maximum. This does not need to be tuned so far

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
        topPidController.setFF(0.000175);
        topPidController.setOutputRange(0.3,1); //min is reverse power minium, max is forward power maximum. This needs to be tuned 

        shootTopController.burnFlash();
        wrist.setSmartCurrentLimit(40); // neo vortex specifications
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.burnFlash();
        //trial and error
        wristEncoder = wrist.getEncoder(); //through bore encoder
        
        // wristPidController.setP(ModuleConstants.P_TURNING);
        // wristPidController.setI(ModuleConstants.I_TURNING);
        // wristPidController.setD(ModuleConstants.D_TURNING);

        // SmartDashboard.putNumber("Turning P", ModuleConstants.P_WRIST);
        // SmartDashboard.putNumber("Turning I", ModuleConstants.I_WRIST);
        // SmartDashboard.putNumber("Turning D", ModuleConstants.D_WRIST);

        
        



        
        //decide pid values later, P, I, D
        pidWrist = new PIDController(0.055, 0.013, 0);

    //   pidWrist = wrist.getPIDController();


        pidWrist.setP(.055);
        pidWrist.setI(.013);
        pidWrist.setD(0);

        

        


        madyannPos = 0.85;
        SmartDashboard.putNumber("P Wrist", 0.055);
        SmartDashboard.putNumber("I Wrist", 0.013);
        SmartDashboard.putNumber("D Wrist", 0.000);
        SmartDashboard.putNumber("Wrist Motor Speed", 0.25);

        SmartDashboard.putNumber("P Bottom Shooter", 0.1);
        SmartDashboard.putNumber("I Bottom Shooter", 0.0);
        SmartDashboard.putNumber("D Bottom Shooter", 0.000);

        SmartDashboard.putNumber("P Top Shooter", 0.1);
        SmartDashboard.putNumber("I Top Shooter", 0.0);
        SmartDashboard.putNumber("D Top Shooter", 0.000);

        //correct value
        wristAbsEncoder = new DutyCycleEncoder(8); // this is a through bore encoder
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
        angle = MathUtil.clamp(angle, 0.8, .97);
        //angle is from .8 to ~.96
        //set tolerance sets the error value to stop the pid loop at 
        double wristPower = pidWrist.calculate(getAbsWristPosition()*100, angle*100);
        moveWrist(wristPower);
        
    }

    public double setShootTarget(double rpm) {
        var temp = shootTarget;
        shootTarget = rpm;
        SmartDashboard.putNumber("Shoot target", shootTarget);
        return temp;
    }

    // public void setShoot(double rpm){
    //     rpm = MathUtil.clamp(rpm, 0, 3000);

    //     shootSpeed = pidShoot.calculate(getLeftSpeed(), rpm);
    //     runShoot(shootSpeed);

    // }

     public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.3, 0.3);
        wrist.set(power);
    }

    public double absToDegrees(double amongus){
        return 0.96-((MAX_DEGREES)-amongus)/(MAX_DEGREES - MIN_DEGREES)*(0.16);
    }

    public double getMadyannsNumber(){
        return madyannPos;
    }

    public void runShoot() {
        shootBottomController.set(-shootSpeed);
        shootTopController.set(-shootSpeed); //inversed the direction in rev
    }

    public void runShoot(double setpoint) {
        // shootBottomController.set(.2); // positive power runs the note out of robot (wheels out), negative power runs the note in to the robot
        // shootTopController.set(.2); // positive power runs the note out of robot (wheels in), negative power runs the note in to the robot
        topPidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        bottomPidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }

    // public void runShoot(double shootSpeed1) {
    //     SmartDashboard.putNumber("Shooting Power for Tuning 1", -shootSpeed1);
    //     shootBottomController.set(-shootSpeed1);
    //     shootTopController.set(-shootSpeed1); //inversed the direction in rev
    // }
    

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

    public double getRPM(){
        return wristAbsEncoder.getFrequency() * 60;
    }

    public void setSpeed(){
        shootSpeed = 3.6 * (getRPM()/5252);
    }

    public double getTopSpeedRPM(){
        return shootEncoderTop.getVelocity();
    }

    public double getBottomSpeedRPM(){
        return shootEncoderBottom.getVelocity();
    }

    public void holdPosition(){
        // calculated line of best fit from tested points
          // moveWrist(-0.1334*getAbsWristPosition() + 0.1343);
        moveWrist(-0.1401*getAbsWristPosition() + 0.141);
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

        // SmartDashboard.putNumber("Wrist Encoder Position", getWristPosition()); 
        SmartDashboard.putNumber("Wrist Encoder ABS Position", getAbsWristPosition()); 

        pidWrist.setP(SmartDashboard.getNumber("P Wrist", .055));
        pidWrist.setI(SmartDashboard.getNumber("I Wrist", 0.013));
        pidWrist.setD(SmartDashboard.getNumber("D Wrist", 0.000));

        bottomPidController.setP(SmartDashboard.getNumber("P Bottom Shooter", 0.1));
        bottomPidController.setI(SmartDashboard.getNumber("I Bottom Shooter", 0));
        bottomPidController.setD(SmartDashboard.getNumber("D Bottom Shooter", 0.000));

        topPidController.setP(SmartDashboard.getNumber("P Top Shooter", 0.1));
        topPidController.setI(SmartDashboard.getNumber("I Top Shooter", 0));
        topPidController.setD(SmartDashboard.getNumber("D Top Shooter", 0.000));
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
        // shootTarget = SmartDashboard.getNumber("Shoot target", getWristPosition());

        SmartDashboard.putNumber("Shoot Bottom Encoder CPR", shootEncoderBottom.getCountsPerRevolution());
        SmartDashboard.putNumber("Shoot Bottom RPM", getBottomSpeedRPM());
        SmartDashboard.putNumber("Shoot Bottom Encoder Position", shootEncoderBottom.getPosition());

        SmartDashboard.putNumber("Shoot Top Encoder CPR", shootEncoderTop.getCountsPerRevolution());
        SmartDashboard.putNumber("Shoot Top RPM", getTopSpeedRPM());
        SmartDashboard.putNumber("Shoot Top Encoder Position", shootEncoderTop.getPosition());
    }
}