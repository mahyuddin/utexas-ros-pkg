package edu.utexas.ece.pharos.proteus3.mobilityPlanes;

public interface MobilityPlane {

  /**
   * Sets the steering angle in radians.  Zero is straight ahead, positive is to
   * the left, and negative is to the right.
   */
  public void setSteeringAngle(double steeringAngle);

  /**
   * Sets the speed in m/s.
   */
  public void setSpeed(double speed);


  /**
   * Stops the robot.
   */
  public void stop();
}
