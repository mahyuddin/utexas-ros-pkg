package edu.utexas.ece.pharos.proteus3.sensors;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;

import proteus3_compass.CompassMsg;

/**
 * Buffers incoming compass measurements.
 *
 * @author Chien-Liang Fok
 */
public class CompassBuffer implements MessageListener<CompassMsg> {
  private Log log;

  public CompassBuffer(Log log) {
    this.log = log;
  }

 @Override
 public void onNewMessage(CompassMsg message) {
        log.info("Received compass message: heading = " + message.getHeading()
          + ", pitch = " + message.getPitch()
          + ", roll = " + message.getRoll());
 }
}
