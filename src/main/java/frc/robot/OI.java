package frc.robot;

import java.util.HashMap;

import frc.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();


    public XboxController driveController, manipController;

    public OI() {
        initControllers();

    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

    }


}