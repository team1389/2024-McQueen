package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;

public class RunElevatorUp extends Command{
    private Elevator elevator;
    Timer timer = new Timer();
    double time = 15;
    
    public RunElevatorUp(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator); 
    }

    public RunElevatorUp(Elevator elevator, double time){
        this.elevator = elevator;
        addRequirements(elevator); 
        this.time = time;
    }

    @Override
    public void execute(){
        elevator.moveToTop();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }
    @Override
    public boolean isFinished(){
        return timer.hasElapsed(time);
    }

}