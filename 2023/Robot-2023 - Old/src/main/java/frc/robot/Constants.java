// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

  public static final int CAN_LEFT_A  = 14;
  public static final int CAN_LEFT_B  = 16; //modified on Sep 15 2022
  public static final int CAN_RIGHT_A = 15;
  public static final int CAN_RIGHT_B = 17;

  public static final int CAN_ROTATE = 20; 
  public static final int CAN_GRIPPER = 21; 
  public static final int CAN_VERTICAL = 18; 
  public static final int CAN_EXTEND = 19; 

  //unused
  public static final int PDP_LEFT_A  = 13; //replace these with where you plug the motors in
  public static final int PDP_LEFT_B  = 14;
  public static final int PDP_RIGHT_A =  3;
  public static final int PDP_RIGHT_B =  2;

  public static final int USB_DRIVER_CONTROLLER   = 0; //what ports you plug the controllers into
  public static final int USB_OPERATOR_CONTROLLER = 1;

  public static final double HAND_SPEED = 0.5; //tune to speed for hand open/closing
  public static final double ROTATE_THROTTLE = 0.5; //tune for speed throttling
  public static final double V_ROTATE_THROTTLE = 0.5; //tune for vertical speed throttling

  public static final double FORWARD_DEADBAND  = 0.05;
  public static final double TURN_DEADBAND     = 0.1;
  public static final double MAX_THROTTLE      = 1.0;
  public static final double TURN_THROTTLE     = 0.8;
  public static final double DEFAULT_RAMP_BAND = 0.1;

  public static final int    LINEAR            = 1;
  public static final int    QUADRATIC         = 2;
  public static final int    CUBIC             = 3;
  public static final int    QUARTIC           = 4;
  public static final int    FORWARD_SENSITIVITY = QUADRATIC;
  public static final int    TURN_SENSITIVITY    = CUBIC;

  //button names do not touch
  public static final int A_BUTTON = 2;
  public static final int B_BUTTON = 3;
  public static final int X_BUTTON = 1;
  public static final int Y_BUTTON = 4;
  public static final int LB = 5;//not working
  public static final int RB = 6;//not working
  public static final int LT = 7;
  public static final int RT = 8;
  public static final int BACK_BUTTON = 9;
  public static final int START_BUTTON = 10;
  public static final int L3 = 11;
  public static final int R3 = 12;
  public static final int LX = 0;
  public static final int LY = 1;
  public static final int RX = 2;
  public static final int RY = 3;
  
} // end class Constants
