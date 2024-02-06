package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;

public class RunElevatorUp extends Command{
    private Elevator elevator;
    double timeout = -1;
    Timer timer = new Timer();
    double time;
    
 
    public RunElevatorUp(Elevator theelevator,double thetime){
        this.elevator = theelevator;
        addRequirements(elevator); 
        time = thetime;
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
