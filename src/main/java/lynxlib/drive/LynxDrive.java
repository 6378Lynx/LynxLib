package lynxlib.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lynxlib.util.CheesyPID;
import lynxlib.util.exceptions.RequiredControllerNotPresentException;
import lynxlib.util.SetpointRamp;

import java.util.function.Function;

import static java.util.Objects.requireNonNull;

public class LynxDrive {

    private final SpeedController leftMotor;
    private final SpeedController rightMotor;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private CheesyPID leftVelocityPID;
    private CheesyPID rightVelocityPID;

    private SetpointRamp leftRamp = new SetpointRamp();
    private SetpointRamp rightRamp = new SetpointRamp();

    private Gyro gyro;
    private CheesyPID gyroPID;

    private boolean gyroStabilizationEnabled = false;
    private boolean closedLoopControlEnabled = false;

    private boolean inLoop;

    private Function<Double, Double> curve = Function.identity();
    private Function<Double, Double> fwdScale = x -> Math.copySign(x * x, x);
    private Function<Double, Double> rotScale = x -> Math.copySign(x * x, x);

    private double kV;
    private double kS;

    private double kDeadBand = 0.1;
    private double kMaxOutput = 0.8;
    private double kMaxSpeed = 1;
    private double kGyroTurnThreshold = 0.1;
    private double kTankDriveGyroTolerance = 0.1;

    /**
     * Constuct a LynxDrive
     *
     * @param leftMotor  SpeedController left motor, SpeedControllerGroup for more than one motor
     * @param rightMotor SpeedController right motor, SpeedControllerGroup for more than one motor
     */
    public LynxDrive(SpeedController leftMotor, SpeedController rightMotor) {
        requireNonNull(leftMotor);
        requireNonNull(rightMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftRamp.setMaxChangePerSecond(3);
        rightRamp.setMaxChangePerSecond(3);

    }

    /**
     * Arcade drive method using LynxDrive
     *
     * @param forward  robot's speed along the X axis, forward is positive
     * @param rotation robot's rotation, clockwise is positive
     */
    public void arcadeDrive(double forward, double rotation) {

        forward = scaleJoystickInput(forward, fwdScale);
        rotation = scaleJoystickInput(rotation, rotScale);

        double left = forward + rotation;
        double right = forward - rotation;

        left = leftRamp.rampValue(left);
        right = rightRamp.rampValue(right);

        if (gyroStabilizationEnabled) {
            double stabilizationOutput = applyGyroStabilization(rotation > 0);
            left += stabilizationOutput;
            right -= stabilizationOutput;
        }

        setOutput(left, right);
    }

    /**
     * Curvature drive method using LynxDrive
     *
     * @param forward   robot's speed along the x-axis
     * @param rotation  robot's rotation, gets applied along a curvature radius
     * @param quickTurn If true, overrides curvature and allows for arcade type movement
     */
    public void curvatureDrive(double forward, double rotation, boolean quickTurn) {
        double left;
        double right;
        forward = scaleJoystickInput(forward, fwdScale);
        rotation = scaleJoystickInput(rotation, rotScale);


        if (forward == 0) {
            quickTurn = true;
        }

        if (quickTurn) {
            left = forward + rotation;
            right = forward - rotation;
        } else {
            left = forward + curve.apply(forward) * rotation;
            right = forward - curve.apply(forward) * rotation;
        }

        left = leftRamp.rampValue(left);
        right = rightRamp.rampValue(right);

        if (gyroStabilizationEnabled) {
            double stabilizationOutput = applyGyroStabilization(rotation > 0);
            left += stabilizationOutput;
            right -= stabilizationOutput;
        }

        double[] normalSpeed = normalizeSpeeds(left, right);

        left = normalSpeed[0];
        right = normalSpeed[1];

        setOutput(left, right);
    }

    /**
     * Tank drive method using LynxDrive
     *
     * @param left  left Speed
     * @param right right Speed
     */
    public void tankDrive(double left, double right) {
        left = scaleJoystickInput(left, fwdScale);
        right = scaleJoystickInput(right, fwdScale);

        left = leftRamp.rampValue(left);
        right = rightRamp.rampValue(right);

        if (gyroStabilizationEnabled) {
            double stabilizationOutput = applyGyroStabilization(Math.abs(left - right) > kTankDriveGyroTolerance);
            left += stabilizationOutput;
            right -= stabilizationOutput;
        }


        setOutput(left, right);
    }

    /**
     * Gives the motor the correct output
     * If closedLoopControl is enabled, converts inputs into speeds and feeds them to a PID, where the output is calculated
     *
     * @param left  the input given to the left motor
     * @param right the input given to the right motor
     */
    private void setOutput(double left, double right) {

        if (closedLoopControlEnabled) {
            left = scaleMaxSpeed(left);
            right = scaleMaxSpeed(right);

            leftVelocityPID.setSetpoint(left);
            rightVelocityPID.setSetpoint(right);

            left = leftVelocityPID.calculate(leftEncoder.getRate(), 0.02) + calculateFractionalFeedForward(left);
            right = leftVelocityPID.calculate(leftEncoder.getRate(), 0.02) + calculateFractionalFeedForward(right);

        }

        left = Math.max(-kMaxOutput, Math.min(kMaxOutput, left));
        right = Math.max(-kMaxOutput, Math.min(kMaxOutput, right));

        leftMotor.set(left);
        rightMotor.set(right);
    }

    /**
     * Calculates a the fractional out with kV and V-Intercept
     *
     * @param speed speed
     * @return fractional out to be set by a motor
     */
    private double calculateFractionalFeedForward(double speed) {
        if (speed == 0) {
            return 0;
        } else {
            return (speed * kV + Math.copySign(kS, speed)) / 12;
        }
    }


    /**
     * Applies Gyro Stabilization to keep the robot driving straight when enabled
     *
     * @param isTurnCommanded if the robot is being commanded to turn
     * @return the output for a speed differential between the left and right sides of the drivetrain
     */
    private double applyGyroStabilization(boolean isTurnCommanded) {
        double output = 0;

        if (!isTurnCommanded) {
            if (!inLoop && Math.abs(gyro.getRate()) < kGyroTurnThreshold) {
                gyroPID.setSetpoint(gyro.getAngle());
                inLoop = true;
            } else {
                output = gyroPID.calculate(gyro.getAngle(), 0.02);
            }

        } else {
            inLoop = false;
            output = 0;
        }
        return output;
    }

    /**
     * Prepares user joystick input to be given to the drive methods
     * Deadbands and clamps to [-maxOutput, maxOutput], scales by user-specified appropriate scaling functions
     *
     * @param input the input to be scaled
     * @param scale the scaling function used to scale input
     * @return the scaled input
     */
    private double scaleJoystickInput(double input, Function<Double, Double> scale) {
        if (Math.abs(input) < kDeadBand) {
            input = 0;
        } else {
            input = Math.max(-kMaxOutput, Math.min(kMaxOutput, input));
            input = scale.apply(input);
        }
        return input;
    }

    /**
     * Scales the input by a user-specified max speed to be used for closed loop control
     *
     * @param input the input
     * @return the scaled input
     */
    private double scaleMaxSpeed(double input) {
        return input * kMaxSpeed;
    }

    /**
     * Normalizes the speed when using Curvature drive
     *
     * @param left  the left speed
     * @param right the right speed
     * @return normalized left and right speed
     */
    private double[] normalizeSpeeds(double left, double right) {
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if (maxMagnitude > 1) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
        return new double[]{left, right};
    }


    /**
     * Sets the left and right encoder
     * Distance per pulse and inversion should be set for the object before passing it in
     *
     * @param leftEncoder  leftside drivetrain Encoder
     * @param rightEncoder rightside drivetrain Encoder
     */
    public void setEncoders(Encoder leftEncoder, Encoder rightEncoder) {
        requireNonNull(leftEncoder);
        requireNonNull(rightEncoder);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    /**
     * Sets the Left and Right drivetrain Velocity PID's
     *
     * @param leftPID  left drivetrain Velocity PID
     * @param rightPID right drivetrain Velocity PID
     */
    public void setVelocityPIDs(CheesyPID leftPID, CheesyPID rightPID) {
        requireNonNull(leftPID);
        requireNonNull(rightPID);
        leftVelocityPID = leftPID;
        rightVelocityPID = rightPID;
    }

    public void setMaxChangePerSecond(double changePerSecond) {
        leftRamp.setMaxChangePerSecond(changePerSecond);
        rightRamp.setMaxChangePerSecond(changePerSecond);
    }

    /**
     * Sets the Gyro Object to be used for Gyro Stabilization
     *
     * @param gyro the gyro object
     */
    public void setGyro(Gyro gyro) {
        requireNonNull(gyro);
        this.gyro = gyro;
    }

    /**
     * Sets the Gyro PID to be used for Gyro Stabilization
     *
     * @param gyroPID the Gyro PID
     */
    public void setGyroPID(CheesyPID gyroPID) {
        requireNonNull(gyroPID);
        this.gyroPID = gyroPID;
        this.gyroPID.setContinuous(true);
        this.gyroPID.setInputRange(0, 360);
    }

    /**
     * Sets the max speed to be scaled to when using closed loop control
     *
     * @param speed max speed - same unit as encoder distance per pulse
     */
    public void setMaxSpeed(double speed) {
        kMaxSpeed = speed;
    }

    /**
     * Sets the constants to be used in the feedforward calculations
     *
     * @param kV         kV
     * @param vIntercept vIntercept
     */
    public void setFeedForwardConstants(double kV, double vIntercept) {
        this.kV = kV;
        this.kS = vIntercept;
    }

    /**
     * Sets the tolerance for gyro stabilization in tank drive
     *
     * @param tolerance the tolerance
     */
    public void setkTankDriveGyroTolerance(double tolerance) {
        kTankDriveGyroTolerance = tolerance;
    }

    /**
     * Sets the joystick deadband
     *
     * @param deadBand the deadband
     */
    public void setkDeadBand(double deadBand) {
        kDeadBand = deadBand;
    }

    /**
     * Sets the threshold for how much gyro turn counts as actually turning
     *
     * @param threshold the threshold
     */
    public void setkGyroTurnThreshold(double threshold) {
        kGyroTurnThreshold = threshold;
    }

    /**
     * Sets the state of closedLoopControl
     * Only can be enabled after passing in both side PID's and Encoders
     *
     * @param enabled state of closed loop control
     */
    public void setClosedLoopEnabled(boolean enabled) {
        if (enabled) {
            verifyClosedLoopComponents();
        }
        closedLoopControlEnabled = enabled;
    }

    /**
     * Sets the state  of Gyro Stabilization
     * Can only be enabled after passing in both a Gyro PID and Gyro Object
     *
     * @param enabled state of gyro stabilization
     */
    public void setGyroStabilizationEnabled(boolean enabled) {
        if (enabled) {
            verifyGyroStabilizationComponents();
        }
        gyroStabilizationEnabled = enabled;
    }

    /**
     * Verifies if components for closed loop control are set
     * Throws RequiredControllerNotPresentException if one is missing
     */
    private void verifyClosedLoopComponents() {
        if (leftVelocityPID == null || rightVelocityPID == null ||
                leftEncoder == null || rightEncoder == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Closed Loop Control Missing \n" +
                            "Set Encoders and Velocity PID Objects"
            );
        }

    }

    /**
     * Verifies if components for gyro stabilization is set
     * Throws RequiredControllerNotPresentException if one is missing
     */
    private void verifyGyroStabilizationComponents() {
        if (gyro == null || gyroPID == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Gyro Stabilization Missing \n" +
                            "Set Gyro and GyroPID objects"
            );
        }

    }

}
