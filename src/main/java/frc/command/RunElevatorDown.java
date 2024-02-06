package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;

public class RunElevatorDown extends Command{
    private Elevator elevator;
    double timeout = -1;
    Timer timer = new Timer();
    double time;
    
 
    public RunElevatorDown(Elevator theelevator){
        this.elevator = theelevator;
        addRequirements(elevator); 
        time = 10;
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
