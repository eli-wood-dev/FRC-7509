// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

  public static final int CAN_LEFT_A  = 9;
  public static final int CAN_LEFT_B  = 10; //modified on Sep 15 2022
  public static final int CAN_RIGHT_A = 4;
  public static final int CAN_RIGHT_B = 5;
  public static final int CAN_ARM     = 8;    
  public static final int CAN_SHOOTER = 6; // DONE: need to change this, check CAN ID on shooter arm

  public static final int PDP_LEFT_A  = 13;
  public static final int PDP_LEFT_B  = 14;
  public static final int PDP_RIGHT_A =  3;
  public static final int PDP_RIGHT_B =  2;
  public static final int PDP_ARM     = 12;    
  public static final int PDP_SHOOTER = 15;

  public static final int USB_DRIVER_CONTROLLER   = 0;
  public static final int USB_OPERATOR_CONTROLLER = 1;

  public static final double ARM_HOLD_UP     =  0.02;
  public static final double ARM_HOLD_DOWN   = -0.07;

  public static final double ARM_UP_LIMIT    =  -2.0; // up position is nominally zero and any arm down
  public static final double ARM_DOWN_LIMIT  = -18.0; // situations result in a negative value

  public static final double ARM_TRAVEL_UP   =  0.30;
  public static final double ARM_TRAVEL_DOWN = -0.23;
  
  //public static final double ARM_TIME_UP   = 0.50;
  //public static final double ARM_TIME_DOWN = 0.35;

  public static final double FORWARD_DEADBAND  = 0.05; // EW changed back to 0.05 to make it easier to drive straight
  public static final double TURN_DEADBAND     = 0.1;  // EW added turning deadband
  public static final double MAX_THROTTLE      = 1.0; // TO DO: let's optimize this for optimum driving speed
                                                       // being mindful of tipping risk (high center of mass)
  public static final double TURN_THROTTLE     = 0.8; // TO DO: let's optimize this for easier slow speed turning
  public static final double DEFAULT_RAMP_BAND = 0.1; //EW 04 Oct 2022

  public static final double INTAKE_SPEED = -0.7;     //ZS Added: 29 Sep 2022
  public static final double SHOOTER_SPEED = 1.0;     //ZS EW Updated: 28 Oct 2022
  public static final double INTAKE_RAMP   = 0.5;     //EW Added: 03 Oct 2022

  public static final int    LINEAR            = 1;
  public static final int    QUADRATIC         = 2;
  public static final int    CUBIC             = 3;
  public static final int    QUARTIC           = 4;
  public static final int    FORWARD_SENSITIVITY = QUADRATIC;
  public static final int    TURN_SENSITIVITY    = CUBIC;
  
} // end class Constants
