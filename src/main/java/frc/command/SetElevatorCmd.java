package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class SetElevatorCmd extends Command{
    ElevatorSubsystem elevator;
    double pos = 2;

    public SetElevatorCmd(ElevatorSubsystem elevator, double pos){
        this.elevator = elevator;
        this.pos = pos;
    }

    @Override
    public void initialize(){
        elevator.controllerInterrupt = false;
     //   elevator.setElevator(pos);
        
    }

    @Override
    public void execute(){
      //  elevator.setElevatorPosBySpeed(pos);
      //  elevator.setSetpoint(pos);
      //  elevator.setElevator(pos);
    }

    // @Override
    // public void end(boolean interrupted){
    //     elevator.stop();
    // }

    @Override
    public boolean isFinished(){
        return true;
    }

}
