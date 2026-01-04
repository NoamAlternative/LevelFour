package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX motor = ArmConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLE);
    private final ArmFeedforward ff = ArmConstants.FEED_FORWARD;
    private final double prevSetpointVel = 0.0;
    private final TrapezoidProfile.State goal = ArmConstants.TARGET_PROFILE_STATE;
    private final TrapezoidProfile.State currentState = ArmConstants.CURRENT_STATE ;
    private double lastAngleMotorProfileGenerationTime;
    private TrapezoidProfile angleMotorProfile = null;

    public Arm() {
        double currentRad = getCurrentAngle().getRadians();
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        setTargetVoltage(calculatePIDOutput(targetAngle));
    }

    void stop() {
        motor.stopMotor();
    }

    private void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private double calculatePIDOutput(Rotation2d targetAngle) {
        return ArmConstants.PID_CONTROLLER.calculate(getCurrentAngle().getRotations(), targetAngle.getRotations());
    }

    private Rotation2d getCurrentAngle(){
        double rotations = ArmConstants.ANGLE_STATUS_SIGNAL.refresh().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }

    private void generateAngleMotorProfile(){
        angleMotorProfile = new TrapezoidProfile(ArmConstants.PROFILE_CONSTRAINTS);
        lastAngleMotorProfileGenerationTime = Timer.getFPGATimestamp();
    }

    private double getAngleMotorProfileTimer(){
        return Timer.getFPGATimestamp() - lastAngleMotorProfileGenerationTime;
    }

    private void setTargetAngleFromProfile(){
        generateAngleMotorProfile();
        if (angleMotorProfile == null){
            motor.stopMotor();
            return;
        }

        TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTimer(),currentState,goal);
        calculatePIDOutput(Rotation2d.fromRadians(targetState.position));
    }
}