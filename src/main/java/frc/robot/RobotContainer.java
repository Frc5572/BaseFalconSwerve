// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.ExampleAuto;
import frc.robot.autos.LimelightAuto;
import frc.robot.autos.ResnickAuto;
import frc.robot.autos.UltrasonicAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.other.Ultrasonic;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private Command autoCommand;

    private static final String exampleAuto = "Example Auto";
    private static final String ultrasonicAuto = "Ultrasonic Auto";
    private static final String limelightAuto = "Limelight Auto";
    private static final String resnickAuto = "Resnick Auto";


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro =
        new JoystickButton(driver, XboxController.Button.kY.value);



    boolean fieldRelative;
    boolean openLoop;


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private Vision vision = new Vision();

    private Ultrasonic ultrasonic = new Ultrasonic();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        this.fieldRelative = Constants.Swerve.isFieldRelative;
        this.openLoop = Constants.Swerve.isOpenLoop;
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, vision, driver, translationAxis,
            strafeAxis, rotationAxis, fieldRelative, openLoop));
        autoChooser.setDefaultOption("Example Auto", exampleAuto);
        autoChooser.addOption("Ultrasonic Auto", ultrasonicAuto);
        autoChooser.addOption("Limelight Auto", limelightAuto);
        autoChooser.addOption("Resnick Auto", resnickAuto);
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {

        if (autoChooser.getSelected() == "Example Auto") {
            System.out.println("Example Auto!!!!!!!!!!!!!!");
            autoCommand = new ExampleAuto(s_Swerve);
        } else if (autoChooser.getSelected() == "Ultrasonic Auto") {
            System.out.println("Ultrasonic Auto!!!!!!!!!!!!!!");
            autoCommand = new UltrasonicAuto(s_Swerve, ultrasonic);
        } else if (autoChooser.getSelected() == "Limelight Auto") {
            System.out.println("Limelight Auto!!!!!!!!!!!!!!");
            autoCommand = new LimelightAuto(s_Swerve, vision);
        } else if (autoChooser.getSelected() == "Resnick Auto") {
            autoCommand = new ResnickAuto(s_Swerve);
        }
        return autoCommand;

    }
}
