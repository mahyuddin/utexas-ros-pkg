package edu.utexas.ece.pharos.demo.homeAutomation;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

import edu.utexas.ece.pharos.brace.Brace;
import edu.utexas.ece.pharos.utils.ByteUtils;
import edu.utexas.ece.pharos.utils.FileLogger;
import edu.utexas.ece.pharos.utils.Logger;
import edu.utexas.ece.pharos.utils.ThreadUtils;

// The following classed are automatically generated. 
// They are located in: 
// ~/.ros/rosjava/lib/org.ros.rosjava.lightswitch_node-0.0.0.jar
import org.ros.message.lightswitch_node.AmbientLight;
import org.ros.message.lightswitch_node.LightSwitchCmd;

/**
 * This is the basic light switch app that tests immediate assertions.
 * It turns off the light, and then
 * turns it back on.  Immediate assertions are used to verify the brightness
 * of the room at different points in the program.
 * 
 * @author Chien-Liang Fok
 */
public class LightSwitchAppImmediate implements NodeMain {

	
	@Override
	public GraphName getDefaultNodeName() {
		// The parameter is the node's name.
		return new GraphName("lightswitch_app/LightSwitchApp");
	}
	
	@Override
	public void onStart(final Node node) {
		
		Logger.setFileLogger(new FileLogger("LightSwitchAppImmediate.log"));
		
		Logger.log("Creating a LightSwitchCmd publisher...");
		final Publisher<LightSwitchCmd> publisher =
				node.newPublisher("lightswitch_cmd", "lightswitch_node/LightSwitchCmd");

		// Obtain a mapped logical variable.  
		// This is normally done using Brace.map(...) and hidden from the
		// application but for now we create it directly.
		MLVAmbientLight mlvAmbientLight = new MLVAmbientLight();
		
		Logger.log("Subscribing to AmbientLight messages...");
		Subscriber<AmbientLight> subscriber = node.newSubscriber("lightLevel", "lightswitch_node/AmbientLight");
		subscriber.addMessageListener(mlvAmbientLight);
		
		// Create the predicates:
		final CPSPredicateRoomBright predicateBright = new CPSPredicateRoomBright(mlvAmbientLight);
		final CPSPredicateRoomDim predicateDim = new CPSPredicateRoomDim(mlvAmbientLight);
		
		// Obtain reference to Brace
		final Brace brace = Brace.getInstance();

		Logger.log("Sleeping for 1s before starting...");
		ThreadUtils.delay(1000);
		node.executeCancellableLoop(new CancellableLoop() {
			
			@Override
			protected void setup() {
			}

			@Override
			protected void loop() {
				LightSwitchCmd cmd = new LightSwitchCmd();
				long startTime, endTime;
				
				Logger.log("Asserting that the room is bright...");
				brace.assertImmediate(predicateBright, 1000, false);
				
				Logger.log("Turning light off...\n");
				cmd.cmd = 0;
				startTime = System.nanoTime();
				publisher.publish(cmd);
				endTime = System.nanoTime();
				Logger.log("Light should be off, latency = " + (endTime - startTime) + "\n");

				Logger.log("Pausing for 1s to allow actuation to take effect...\n");
				ThreadUtils.delay(1000);
				Logger.log("Asserting that the room is dim...");
				brace.assertImmediate(predicateDim, 1000, false);
				
				Logger.log("Waiting 4s...\n");
				ThreadUtils.delay(4000);

				Logger.log("Turning light on...\n");
				cmd.cmd = 1;
				startTime = System.nanoTime();
				publisher.publish(cmd);
				endTime = System.nanoTime();
				Logger.log("Light should be on, latency = " + (endTime - startTime) + "\n");

				Logger.log("Pausing for 1s to allow actuation to take effect...\n");
				ThreadUtils.delay(1000);
				Logger.log("Asserting that the room is bright...");
				brace.assertImmediate(predicateBright, 1000, false);
				
				Logger.log("Done!");
				System.exit(0);
			}
		});
	}

	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
	}
}