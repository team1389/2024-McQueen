package frc.robot;

import java.util.HashMap;

import frc.command.*;
import frc.command.TeleOpDrive;
import frc.subsystems.*;
import frc.subsystems.Indexer;
import frc.util.DPadButton;
import frc.util.DPadButton.Direction;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {


    private GenericHID driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;
    private Trigger driveXButton;
    private Trigger driveBButton;
    private Trigger driveYButton;
    private Trigger driveRightTrigger;
    private Trigger driveLeftTrigger;



    private XboxController manipController;
    private Trigger manipEllipsisButton;
    private Trigger manipMenuButton;
    private Trigger manipFullscreen;
    private Trigger manipStadia;
    private Trigger manipGoogle;


    private Trigger manipRightBumper;
    private Trigger manipLeftBumper;
    private Trigger manipRightTrigger;
    private Trigger manipLeftTrigger;

    private Trigger manipAButton;
    private Trigger manipBButton;
    private Trigger manipXButton;
    private Trigger manipYButton;

    private Trigger manipUp;
    private Trigger manipLeft;
    private Trigger manipDown;
    private Trigger manipRight;
    
    public final Drivetrain drivetrain = new Drivetrain();
    
    public final Indexer larry = new Indexer();

    public final Intake intake = new Intake();
    public OI() {
        
        initControllers();
        manipAButton.whileTrue(new RunIntake(intake));
        manipBButton.onTrue(new RunIndexer(larry));

        // Cool new way to make a drive command by passing in Suppliers for the
        // joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
                drivetrain,
                () -> getDriveLeftY(),
                () -> getDriveLeftX(),
                () -> getDriveRightX(),
                () -> getDriveRightY(),
                () -> getDriveLeftBumper(), // By default be in field oriented
                () -> !getDriveRightBumper(), // Slow function
                () -> driveXButton.getAsBoolean(), // Hold x position
                () -> driveRightTrigger.getAsBoolean(),
                () -> driveController.getRawAxis(5)) // flip
        );

        // Press A button -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);
        manipAButton = new JoystickButton(manipController,0);//change
        


        driveAButton = new JoystickButton(driveController, 1);

    }

    private double getDriveLeftX() {
        return driveController.getRawAxis(0);
    }
    
    private double getDriveLeftY() {
        return driveController.getRawAxis(1);
    }
    
    private double getDriveRightX() {
        return driveController.getRawAxis(3); 
    }
    
    private double getDriveRightY() {
        return driveController.getRawAxis(4); 
    }
    
    private boolean getDriveLeftBumper() {
        return !driveController.getRawButton(5);
    }
    
    private boolean getDriveRightBumper() {
        return !driveController.getRawButton(6);
    }


}