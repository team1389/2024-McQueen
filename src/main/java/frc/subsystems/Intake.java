package frc.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

//1 motor that spins the intake thingies

public class Intake extends SubsystemBase{
    private final double intakeSpeed = 1;
    private CANSparkFlex intakeMotor;
    private AnalogPotentiometer pot;
    private final double distanceWONode = 46.7; //change
    public Intake(){
    intakeMotor = new CANSparkFlex(RobotMap.MotorPorts.INTAKE_MOTOR,MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.burnFlash();
    //incorrect port, needs to be moved to anolog from DIO
    pot = new AnalogPotentiometer(0, 100, 30); //change parameters
    }
    
    public void runIntake(){
        intakeMotor.set(-intakeSpeed);
    }

    public void runOuttake(){
        intakeMotor.set(intakeSpeed);
    }

    public boolean hitSensor(){ // w/o note sensor is at 35.2
        // distance sensor does not get values
        SmartDashboard.putNumber("Distance Sensor", pot.get());
        return distanceWONode - pot.get() > 5;
    }

    public void stop(){
        intakeMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Distance Sensor", pot.get());
    }
}
