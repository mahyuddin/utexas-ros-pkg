package edu.utexas.ece.pharos.proteus3.sensors;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;

// Import the messages
import proteus3_gps.GPSMsg;

/**
 * Buffers incoming GPS measurements.
 *
 * @author Chien-Liang Fok
 */
public class GPSBuffer implements MessageListener<GPSMsg> {
  private Log log;

  public GPSBuffer(Log log) {
    this.log = log;
  }

 @Override
 public void onNewMessage(GPSMsg message) {

  log.info("Received GPS message: timeSec = " + message.getTimeSec()
          + ", timeUsec = " + message.getTimeUsec()
          + ", latitude = " + message.getLatitude()
          + ", longitude = " + message.getLongitude()
          + ", altitude = " + message.getAltitude()
          + ", UtmE = " + message.getUtmE()
          + ", UtmN = " + message.getUtmN()
          + ", quality = " + message.getQuality()
          + ", numSats = " + message.getNumSats()
          + ", Hdop = " + message.getHdop()
          + ", Vdop = " + message.getVdop());
 }
}
