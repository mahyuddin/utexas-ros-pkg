package edu.utexas.ece.pharos.proteus3.mobilityPlanes;

import edu.utexas.ece.pharos.logger.Logger;

public class TraxxasMobilityPlane extends MobilityPlane {

	/**
	 * The constructor.
	 */
	public TraxxasMobilityPlane() {

	}

	/**
	 * Sets the steering angle in radians.  Zero is straight ahead, positive is to
	 * the left, and negative is to the right.
	 */
	@Override
	public void setSteeringAngle(double steeringAngle) {
		Logger.log("Steering angle set to " + steeringAngle);
	}

	/**
	 * Sets the speed in m/s.
	 */
	@Override
	public void setSpeed(double speed) {
		Logger.log("Speed set to " + speed);
	}

	@Override
	public void stop() {
		Logger.log("Stop called.");
	}
}
