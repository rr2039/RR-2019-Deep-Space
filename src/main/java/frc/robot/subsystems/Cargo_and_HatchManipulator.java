/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Cargo_and_HatchManipulator extends Subsystem {
  public void intakeCargo() {
    RobotMap.leftCargoIntake.set(1);
    RobotMap.rightCargoIntake.set(1);
  }

  public void ejectCargo() {
    RobotMap.leftCargoIntake.set(-1);
    RobotMap.rightCargoIntake.set(-1);
  }
  public void cargoIntakeOff() {
    RobotMap.leftCargoIntake.set(0);
    RobotMap.rightCargoIntake.set(0);
  }

  public void grabHatch() {
  }

  public void releaseHatch() {
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
