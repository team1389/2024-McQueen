package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;

public class RunElevatorDown extends Command{
    private Elevator elevator;
    Timer timer = new Timer();
    double time = 15;
    
 
    public RunElevatorDown(Elevator theelevator){
        this.elevator = theelevator;
        addRequirements(elevator); 
    }

    public RunElevatorDown(Elevator theelevator, double time){
        this.elevator = theelevator;
        addRequirements(elevator); 
        this.time = time;
    }

    @Override
    public void execute(){
        elevator.moveToBottom();
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
