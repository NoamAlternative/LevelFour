package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ArmConstants {

    private static final int
            MOTOR_ID = 1,// motor's id
            ENCODER_ID = 1;// encoder's id

    private static final String
            MOTOR_NAME = "ArmMotor",//motor's name
            ENCODER_NAME = "ArmEncoder";// encoder's name

    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    static final double
            GEAR_RATIO = 42,//ill take this as an example when the motor spins 42 times the arm rotates once
            ARM_LENGTH_METERS = 0.52,//the length of the arm
            ARM_MASS_KG = 3.5, //the arm's mass(kg)
            MOTOR_CURRENT_LIMIT = 50; // limits the current(ampere) to 50A

    static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(0),// Minimum angle the arm is allowed to reach
            MAX_ANGLE = Rotation2d.fromDegrees(360);// Minimum angle the arm is allowed to reach

    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0; //the default position of the motor

    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET =
            RobotHardwareStats.isSimulation()
                    ? -Conversions.degreesToRotations(90) // "?" means "if it's true" then that the value of the variable
                    : -ANGLE_ENCODER_GRAVITY_OFFSET; // ":" means "if it's false"

    private static final double
            MAX_VELOCITY = RobotHardwareStats.isSimulation() ? 2.46 : 0, //same here
            MAX_ACCELERATION = RobotHardwareStats.isSimulation() ? 67.2 : 0; //same here

    static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION); //define the constraints

    private static final double
            P = RobotHardwareStats.isSimulation() ? 34 : 0,
            I = 0,
            D = RobotHardwareStats.isSimulation() ? 3 : 0;

    private static final double
            KS = RobotHardwareStats.isSimulation() ? 0.026 : 0,
            KV = RobotHardwareStats.isSimulation() ? 4.87 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0.178 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0.112 : 0;

    static final PIDController PID = new PIDController(P, I, D);
    static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, KV, KA);

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

    static final boolean FOC_ENABLED = true;//who doesn't know wot de FOC is this?

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);//creates a motor model for simulation that representing a real motor

    static final SingleJointedArmSimulation SIMULATION =
            new SingleJointedArmSimulation(
                    GEARBOX,
                    GEAR_RATIO,
                    ARM_LENGTH_METERS,
                    ARM_MASS_KG,
                    MIN_ANGLE,
                    MAX_ANGLE,
                    true
            ); //creates a simulation of a single-jointed arm

    static final SingleJointedArmMechanism2d MECHANISM =
            new SingleJointedArmMechanism2d(
                    "Arm",
                    ARM_LENGTH_METERS,
                    Color.kBlue
            );//defining the mechanism of the arm in the simulation

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO; //configuring the gear ratio
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();// Tells the motor to read from this encoder
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;// the source of the encoder

        config.Slot0.kP = P; // slot is a different settings for the same motor that you can change in the middle of a run
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // defining the type of the gravity for the motor for example the arm gravity is change based on where is it
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign; // calculating and sending power for the motor based on the gravity type(feed forward)

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        config.CurrentLimits.StatorCurrentLimitEnable = true; //is using limited current for the motor
        config.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT; //defining the limit of the current(amper) in this case 50A

        MOTOR.applyConfiguration(config); //applying the configurations
        MOTOR.setPhysicsSimulation(SIMULATION); //setting the physics in the simulation
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration(); //creating a new CANcoderConfiguration

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; //setting the sensor direction to be clockwise
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET; //setting the sensor direction to be the offset position
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.applyConfiguration(config);//apply config
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);
        ENCODER.registerSignal(,100); //update the frequency of the encoder to 100hz witch is 100 milliseconds
        ENCODER.registerSignal(,100); //update the frequency of the encoder to 100hz witch is 100 milliseconds
    }

    public enum ArmState {
        HIGH(Rotation2d.fromDegrees(167)),
        LOW(Rotation2d.fromDegrees(67)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        ArmState(Rotation2d angle) {
            this.targetAngle = angle;
        }
    }
}