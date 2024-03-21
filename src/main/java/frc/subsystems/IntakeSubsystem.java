package frc.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.IntakeConstants;

//1 motor that spins the intake thingies

public class IntakeSubsystem extends SubsystemBase{
    private final double intakeSpeed = 1; //should be 1
    private CANSparkFlex intakeMotor;
    private AnalogPotentiometer intakeDistanceSensor;
    private boolean isNoteIn = true;

    public IntakeSubsystem(){
    intakeMotor = new CANSparkFlex(RobotMap.MotorPorts.INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.burnFlash();
    //The type of distance sensor we have in the intake, a 2m rev IR Distance Sensor, must be declared as an analog potentiometer
    intakeDistanceSensor = new AnalogPotentiometer(0, 100, 30); 
    SmartDashboard.putNumber("Intake Distance Sensor", intakeDistanceSensor.get());
    SmartDashboard.putBoolean("isNoteIn", isNoteIn);
    }
    
    public void runIntake(){
        intakeMotor.set(-intakeSpeed);
    }

    public void runOuttake(){
        intakeMotor.set(.2);
    }

    public boolean hitSensor(){ 
        return isNoteIn;
    }

    public void stop(){
        intakeMotor.set(0);
    }

    @Override
    public void periodic(){
        isNoteIn = intakeDistanceSensor.get() > (IntakeConstants.kDistanceWithoutNode + 5);
        SmartDashboard.putNumber("Intake Distance Sensor", intakeDistanceSensor.get());
        SmartDashboard.putBoolean("isNoteIn", hitSensor());
    }
}
