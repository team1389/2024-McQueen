package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class ManualElevatorCmd extends Command{
    ElevatorSubsystem elevator;
    private Supplier<Double> power;

    public ManualElevatorCmd(ElevatorSubsystem elevator, Supplier<Double> power) {
        this.elevator = elevator;
        this.power = power;
        addRequirements(elevator);
    }
    
    @Override
    public void execute() {
        elevator.controllerInterrupt = true;
        elevator.moveElevator(MathUtil.clamp(power.get(), -1, 1));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.moveElevator(0);
    }
}