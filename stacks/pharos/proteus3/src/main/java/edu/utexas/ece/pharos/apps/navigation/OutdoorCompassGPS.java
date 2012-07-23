package edu.utexas.ece.pharos.apps.navigation;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import proteus3_compass.compass;
import proteus3_gps.gps;

public class OutdoorCompassGPS extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("proteus3_nav_compass_gps/outdoor_compass_gps");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();

    Subscriber<compass> compassSubscriber = connectedNode.newSubscriber("/compass/measurement", compass._TYPE);
    compassSubscriber.addMessageListener(new MessageListener<compass>() {
      @Override
      public void onNewMessage(compass message) {
        log.info("Received compass message: heading = " + message.getHeading()
          + ", pitch = " + message.getPitch()
          + ", roll = " + message.getRoll());
      }
    });

	Subscriber<gps> gpsSubscriber = connectedNode.newSubscriber("/gps/measurement", gps._TYPE);
    gpsSubscriber.addMessageListener(new MessageListener<gps>() {
      @Override
      public void onNewMessage(gps message) {
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
    });

     
  }
}
