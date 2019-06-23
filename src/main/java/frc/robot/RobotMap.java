/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class RobotMap {

  public static WPI_TalonSRX talonFL = new WPI_TalonSRX(0);
  public static WPI_TalonSRX talonBL = new WPI_TalonSRX(3);
  public static WPI_TalonSRX talonFR = new WPI_TalonSRX(15);
  public static WPI_TalonSRX talonBR = new WPI_TalonSRX(13);

  public static WPI_VictorSPX leftCargoIntake = new WPI_VictorSPX(6);
  public static WPI_VictorSPX rightCargoIntake = new WPI_VictorSPX(10);

  public static WPI_TalonSRX liftMotor = new WPI_TalonSRX(14);
  public static  WPI_TalonSRX wristMotor = new WPI_TalonSRX(9);
}