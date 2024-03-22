package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class SetElevatorCmd extends Command{
    ElevatorSubsystem elevatorSubsystem;
    double pos = 2;
    boolean check = true;

    public SetElevatorCmd(ElevatorSubsystem elevatorSubsystem, double pos, boolean check){
        this.elevatorSubsystem = elevatorSubsystem;
        this.pos = pos;
        this.check = check;

    }

    @Override
    public void initialize(){
     //   SmartDashboard.putBoolean("check", check);
      //  elevatorSubsystem.controllerInterrupt = false;
     //   elevatorSubsystem.setElevator(pos);
        
    }

    @Override
    public void execute(){
      //  elevatorSubsystem.setElevatorPosBySpeed(pos);
      //  elevatorSubsystem.setSetpoint(pos);
       // elevatorSubsystem.setElevator(pos);
       if (check){
        if(elevatorSubsystem.getRelEncoderPos() < elevatorSubsystem.high/2){
            
          //  SmartDashboard.putNumber("elevatorSubsystem height", elevatorSubsystem.getRelEncoderPos());
            elevatorSubsystem.moveElevator(.1);
          //  SmartDashboard.putBoolean("check", check);


       } else{
      //  SmartDashboard.putNumber("elevatorSubsystem height", 0);
        check = false;
      //  SmartDashboard.putBoolean("check", check);
        elevatorSubsystem.stop();
       }
    }
}

    @Override
    public void end(boolean interrupted){
      //  SmartDashboard.putBoolean("check", check);
        elevatorSubsystem.stop();
    }

    // @Override
    // public boolean isFinished(){
    //     //if()
    //     return elevatorSubsystem.isPidFinished(pos);
    //     // elevatorSubsystem.controllerInterrupt = true;
    //     // return true;
    // }

}
