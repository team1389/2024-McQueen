package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;

public class SetElevator extends Command{
    Elevator elevator;
    double pos = .5;

    public SetElevator(Elevator elevator, double pos){
        this.elevator = elevator;
        this.pos = pos;
    }

    @Override
    public void execute(){
      //  elevator.setSetpoint(pos);
        elevator.setElevator(pos);
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }

}
