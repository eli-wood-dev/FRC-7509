package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

//Eli Wood

public class Arm {
    CANSparkMax rotate;
    CANSparkMax hand;
    TalonFX vertical;
    TalonFX extend;

    public Arm(CANSparkMax rotate, CANSparkMax hand, TalonFX vertical, TalonFX extend) {
        this.rotate = rotate;
        this.hand = hand;
        this.vertical = vertical;
        this.extend = extend;
    }

    //add vertical rotation and extension

    void extend() {
        extend.set(ControlMode.PercentOutput, 0);
    }

    void retract(double amount){
        extend.set(ControlMode.PercentOutput, amount);
    }

    void vRotate(double amount) {
        vertical.set(ControlMode.PercentOutput, amount*Constants.V_ROTATE_THROTTLE);
    }

    void rotate(double amount) {
        rotate.set(amount*Constants.ROTATE_THROTTLE);

    }

    void close() {
        hand.set(-Constants.HAND_SPEED);
    }

    void open() {
        hand.set(Constants.HAND_SPEED);
    }
}
