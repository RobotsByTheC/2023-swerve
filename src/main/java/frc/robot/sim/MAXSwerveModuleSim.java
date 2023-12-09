package frc.robot.sim;

import static frc.robot.Constants.ModuleConstants.kDrivingMotorReduction;
import static frc.robot.Constants.ModuleConstants.kTurningEncoderPositionFactor;
import static frc.robot.Constants.ModuleConstants.kWheelCircumferenceMeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a MAX Swerve module with a single NEO driving the wheel and a single NEO 550
 * controlling the module's heading.
 */
public class MAXSwerveModuleSim implements MechanismSim {
  // Simulate both the wheel and module azimuth control as simple flywheels
  // Note that this does not perfectly model reality - in particular, friction is not modeled
  // The wheel simulation has a much higher moment of inertia than the physical wheel in order to
  // account for the inertia of the rest of the robot. (This is a /very/ rough approximation)
  private final FlywheelSim wheelSim =
      new FlywheelSim(DCMotor.getNEO(1), kDrivingMotorReduction, 1.25);
  private final FlywheelSim turnSim =
      new FlywheelSim(DCMotor.getNeo550(1), kTurningEncoderPositionFactor, 0.001);

  // PID constants are different in simulation than in real life
  private final PIDController drivePID = new PIDController(9, 0, 0);
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.08, 0.05);
  private final PIDController turnPID = new PIDController(1, 0, 0);

  private final SwerveModulePosition currentPosition = new SwerveModulePosition();

  private SwerveModuleState desiredState = new SwerveModuleState();

  public MAXSwerveModuleSim() {
    turnPID.enableContinuousInput(0, 2 * Math.PI);

    SimulationContext.getDefault().addMechanism(this);
  }

  public void update(double timestep) {
    // Update PID controllers and get their idealized voltage output.
    // Those voltages may not be physically possible (eg trying to output +16V when the battery can
    // only do 12 at most), so we wrap the results in a call to `outputVoltage` to make sure the
    // voltages we feed to the simulated hardware are possible.
    var wheelVoltage = outputVoltage(
        drivePID.calculate(getWheelVelocity())
            + driveFeedForward.calculate(desiredState.speedMetersPerSecond)
    );
    var turnVoltage = outputVoltage(
        turnPID.calculate(getModuleAngle().getRadians())
    );

    // Update the simulations with the new commanded voltages
    wheelSim.setInputVoltage(wheelVoltage);
    turnSim.setInputVoltage(turnVoltage);
    wheelSim.update(timestep);
    turnSim.update(timestep);

    // Write "sensor" values by integrating the simulated velocities over the past timestep
    currentPosition.angle =
        currentPosition.angle.plus(Rotation2d.fromRadians(turnSim.getAngularVelocityRadPerSec() * timestep));
    currentPosition.distanceMeters +=
        wheelSpeedFromAngular(wheelSim.getAngularVelocityRadPerSec()) * timestep;
  }

  @Override
  public double getCurrentDraw() {
    return wheelSim.getCurrentDrawAmps() + turnSim.getCurrentDrawAmps();
  }

  /**
   * Sets the desired state (velocity and module angle) of the simulated module.
   *
   * @param state the desired state
   */
  public void setDesiredState(SwerveModuleState state) {
    this.desiredState = state;
    drivePID.setSetpoint(state.speedMetersPerSecond);
    turnPID.setSetpoint(state.angle.getRadians());
  }

  /**
   * Gets the current linear velocity of the simulated wheel, in meters per second
   */
  public double getWheelVelocity() {
    return wheelSpeedFromAngular(wheelSim.getAngularVelocityRadPerSec());
  }

  /**
   * Gets the total linear distanced travelled by the simulated wheel relative to its own reference
   * frame, in meters.
   */
  public double getDistanceTraveled() {
    return currentPosition.distanceMeters;
  }

  /**
   * Gets the current angle of the simulated module relative to its starting angle.
   */
  public Rotation2d getModuleAngle() {
    return currentPosition.angle;
  }

  /**
   * Resets the simulated distance to 0 meters.
   */
  public void resetDistance() {
    currentPosition.distanceMeters = 0;
  }

  /**
   * Converts an angular wheel velocity into the corresponding tangential linear speed.
   *
   * @param angularWheelVelocity the angular velocity of the wheel, in radians per second.
   * @return the tangential speed of the wheel in meters per second.
   */
  private double wheelSpeedFromAngular(double angularWheelVelocity) {
    return angularWheelVelocity * (2 * Math.PI) * kWheelCircumferenceMeters;
  }
}
