package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class SetElevatorCmd extends Command{
    ElevatorSubsystem elevator;
    double pos = 2;
    boolean check = true;

    public SetElevatorCmd(ElevatorSubsystem elevator, double pos, boolean check){
        this.elevator = elevator;
        this.pos = pos;
        this.check = check;

    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("check", check);
      //  elevator.controllerInterrupt = false;
     //   elevator.setElevator(pos);
        
    }

    @Override
    public void execute(){
      //  elevator.setElevatorPosBySpeed(pos);
      //  elevator.setSetpoint(pos);
       // elevator.setElevator(pos);
       if (check){
        if(elevator.getRelEncoderPos() < elevator.high/2){
            
            SmartDashboard.putNumber("Elevator height", elevator.getRelEncoderPos());
            elevator.moveElevator(.1);
            SmartDashboard.putBoolean("check", check);


       } else{
        SmartDashboard.putNumber("Elevator height", 0);
        check = false;
        SmartDashboard.putBoolean("check", check);
        elevator.stop();
       }
    }
}

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("check", check);
        elevator.stop();
    }

    // @Override
    // public boolean isFinished(){
    //     //if()
    //     return elevator.isPidFinished(pos);
    //     // elevator.controllerInterrupt = true;
    //     // return true;
    // }

}
