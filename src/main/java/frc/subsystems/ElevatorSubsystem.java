// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ModuleConstants;
import frc.robot.RobotMap.ShooterConstants;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase{
    double speed = 1;
    int count = 0;
    public double target = 2;
    public double low = -1.75;
    public double high = low+4.6;
    public double middle = (high+low)/2;
    public double encoderPos;
    private CANSparkFlex elevatorMotor;
    private DutyCycleEncoder elevatorEncoder;
    private SparkFlexExternalEncoder elevatorEncoder1;
    private RelativeEncoder elevatorRelativeEncoder;
    // private SparkAbsoluteEncoder elevatorAbsoluteEncoder;
    private final PIDController elevatorPid;
    private SparkPIDController elevatorPidController;
    public boolean controllerInterrupt = true;
    public SparkPIDController elevatorSparkPid;
//set it to absolute mode

    public ElevatorSubsystem(){
        elevatorMotor = new CANSparkFlex(RobotMap.MotorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.setSmartCurrentLimit(40);

        elevatorRelativeEncoder = elevatorMotor.getEncoder();

        elevatorPidController = elevatorMotor.getPIDController();
        elevatorPidController.setFeedbackDevice(elevatorRelativeEncoder);

        elevatorPidController.setP(0.1);
        elevatorPidController.setI(0);
        elevatorPidController.setD(0);
        // bottomPidController.setIZone(0);
        elevatorPidController.setOutputRange(-.3, .3);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.burnFlash();
        elevatorEncoder = new DutyCycleEncoder(RobotMap.MotorPorts.ELEVATOR_ENCODER);
        elevatorPid = new PIDController(0, 0, 0);
        // elevatorAbsoluteEncoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elevatorEncoder.reset();
     //   elevatorRelativeEncoder.reset();
       // elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // SmartDashboard.putNumber("Elevator P", 1.7);
        // SmartDashboard.putNumber("Elevator I", 0.005);
        // SmartDashboard.putNumber("Elevator D", 0.000);

      elevatorRelativeEncoder.setPosition(elevatorEncoder.getAbsolutePosition());
        
      //  SmartDashboard.putNumber("Elevator Encoder Abs Pos", 0);

        // elevatorRelativeEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    }

    public double setTarget(double pos) {
        var temp = pos;
        target = pos;
        // SmartDashboard.putNumber("Elevator Target", target);
        return temp;
    }


    public void moveElevator(double power){
        elevatorMotor.set(power);
    }

    public double getElevatorPosition(){
        return elevatorEncoder.getAbsolutePosition() - elevatorEncoder.getPositionOffset();
    }

    // public double getAbsElevatorPosition(){
    //     return elevatorAbsoluteEncoder.getPosition();
    // }

    public double getRelEncoderPos(){
        return elevatorRelativeEncoder.getPosition();
    }

    // public double getElevatorPosition(){

    //     return elevatorEncoder.getPosition();
    // }

    public void moveToTop(){
        elevatorMotor.set(0.3);
    }
    public void moveToBottom(){
        elevatorMotor.set(-0.3);
    }
    public void stop(){
        elevatorMotor.set(0.0);
    }

    public boolean isPidFinished(double pos){
        return Math.abs(getRelEncoderPos()-pos) < .01;
    }

    public void setSetpoint(double setpoint){
        target = setpoint;
    }

    public void setElevatorPosBySpeed(double pos){
        //  double value = ((pos-middle)/(middle*2));
        //  elevatorMotor.set(Math.cos(value)*.1);
       // elevatorMotor.set((1-(pos-middle)/middle)*.1);
    //     MathUtil.clamp(elevator)
    elevatorMotor.set(.2);
       // SmartDashboard.putNumber("Elevator height", getRelEncoderPos());

    //    if(pos < high/2 && pos > low/2){
    //     elevatorMotor.set(.1);
    //    } else{
    //     stop();
    //    }
      //  SmartDashboard.putNumber("Elevator value", value);
    //    elevatorMotor.set(.1);
    }

    public void setElevator(double pos){
        elevatorMotor.set(elevatorPid.calculate(getRelEncoderPos(), target));
    }

    public double getElevatorEncoderCount() { //encoder count - test
        double pos = elevatorEncoder.getAbsolutePosition();
        return (pos < 0.5) ? pos + (2*Math.PI) : pos;
    }

    public void HoldElevator(){
        elevatorSparkPid.setReference(getRelEncoderPos(), CANSparkMax.ControlType.kPosition);
    }



    @Override
    public void periodic() {
        // SmartDashboard.putNumber("high", high/2);
        //     SmartDashboard.putNumber("low", low/2);

        if(!controllerInterrupt){
            elevatorMotor.set(elevatorPid.calculate(getRelEncoderPos(), target));
        }

        // SmartDashboard.putNumber("Elevator Rel Encoder", elevatorRelativeEncoder.getPosition());
        // SmartDashboard.putNumber("Elevator Abs Encoder", elevatorEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("Elevator Encoder Distance", elevatorEncoder.getDistance());

      //  SmartDashboard.putNumber("Elevator Encoder Abs Pos", getAbsElevatorPosition());
        // SmartDashboard.putNumber("Elevator Encoder position", getElevatorPosition());

        // SmartDashboard.putNumber("Elevator Pos Offset", elevatorEncoder.getPositionOffset());

        // SmartDashboard.putNumber("elevator speed", Math.cos(1/2));
        // SmartDashboard.putNumber("Elevator Encoder Count", getElevatorEncoderCount());

        // SmartDashboard.putNumber("Elevator Pos", getRelEncoderPos());

        // elevatorPid.setP(SmartDashboard.getNumber("Elevator P", .12));
        // elevatorPid.setI(SmartDashboard.getNumber("Elevator I", 0.005));
        // elevatorPid.setD(SmartDashboard.getNumber("Elevator D", 0.000));

        // if (lastPos < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1 &&
        //     absoluteEncoder.getPosition() > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1) {
        //         counter--;
        //     } 
        //     else if (lastPos > FourBarConstants.SHOULDER_FLIP_MAX.getRadians() - 0.1 && 
        //     absoluteEncoder.getPosition() < FourBarConstants.SHOULDER_FLIP_MIN.getRadians() + 0.1) {
        //         counter++;
        //     }
        //     lastPos = absoluteEncoder.getPosition();
                    
                }
}
