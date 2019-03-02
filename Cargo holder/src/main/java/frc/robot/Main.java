/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.hal.sim.*;
import edu.wpi.first.hal.sim.mockdata.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.command.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
  public class RIMoter ;
  public class LOMoter ;
   
  WPI_TalonSRX RIMoter = new WPI_TalonSRX(20) ; // Need to be changed
  
  