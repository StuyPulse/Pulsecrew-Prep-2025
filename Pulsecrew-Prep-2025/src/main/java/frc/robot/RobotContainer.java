// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autons.DoNothingAuton;
import frc.robot.commands.autons.Mobility;
import frc.robot.commands.tank.TankDriveDrive;
import frc.robot.subsystems.TankDrive;

public class RobotContainer {

  public final TankDrive tankDrive = new TankDrive();

  public final XboxController driver = new XboxController(0);

  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    configureAutons();
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    tankDrive.setDefaultCommand(new TankDriveDrive(tankDrive, driver));
  }

  public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        autonChooser.addOption("Mobility", new Mobility());
    
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
