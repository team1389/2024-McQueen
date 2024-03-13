package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class RunElevatorUpCmd extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    Timer timer = new Timer();
    double time = 6.5;
    
    public RunElevatorUpCmd(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        // addRequirements(elevatorSubsystem); 
    }

    public RunElevatorUpCmd(ElevatorSubsystem elevatorSubsystem, double time){
        this.elevatorSubsystem = elevatorSubsystem;
        // addRequirements(elevatorSubsystem); 
        this.time = time;
    }

    // @Override
    // public void initialize(){
    //     timer.start();
    // }

    @Override
    public void execute(){
        elevatorSubsystem.moveToTop();
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.stop();
    }
    // @Override
    // public boolean isFinished(){
    //     return timer.hasElapsed(time);
    // }

}
