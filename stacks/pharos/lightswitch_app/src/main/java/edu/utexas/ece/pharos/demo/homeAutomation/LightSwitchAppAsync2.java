package edu.utexas.ece.pharos.demo.homeAutomation;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import edu.utexas.ece.pharos.brace.Brace;
import edu.utexas.ece.pharos.brace.CPSAssertion;
import edu.utexas.ece.pharos.utils.FileLogger;
import edu.utexas.ece.pharos.utils.Logger;
import edu.utexas.ece.pharos.utils.ThreadUtils;

// The following classed are automatically generated. 
// They are located in: 
// ~/.ros/rosjava/lib/org.ros.rosjava.lightswitch_node-0.0.0.jar
import lightswitch_node.AmbientLight;
import lightswitch_node.LightSwitchCmd;

/**
 * This is the basic light switch app that tests asynchronous assertions.
 * It turns off the light, and then
 * turns it back on.  Asynchronous assertions are used to verify the brightness
 * of the room at different points in the program.
 * 
 * This application implements the following pseudo-code:
 * 
 * <pre>
 * assert-async room_brightness is high
 * 
 * lightswitch <-- OFF
 * delay 0.5 seconds
 * 
 * a = assert-async room_brightness is low
 * 
 * delay 4 seconds
 * 
 * lightswitch <-- ON
 * delay 0.5 seconds
 * 
 * a = assert-async room_brightness is high
 * </pre>
 * 
 * @author Chien-Liang Fok
 */
public class LightSwitchAppAsync2 extends AbstractNodeMain {

	/**
	 * The amount of time in milliseconds to allow the actuation
	 * command to effect the physical environment.
	 */
	public static final long ACTUATION_PAUSE_TIME = 500;
	
	/**
	 * The final pause time after the end of the program
	 * to allow the final assertion to be evaluated.
	 */
	public static final long FINAL_PAUSE_TIME = 1000;
	
	/**
	 * The max amount of time in milliseconds between when a sensor
	 * is accessed and when it should have been accessed.
	 */
	public static final long MAX_DELTA = 200;
	
	/**
	 * The max amount of time in milliseconds that can pass before
	 * the assertion must run.
	 */
	public static final long MAX_LATENCY = 1000;
	
	@Override
	public GraphName getDefaultNodeName() {
		// The parameter is the node's name.
		return GraphName.of("lightswitch_app/LightSwitchApp");
	}
	
	@Override
	public void onStart(ConnectedNode node) {
		
		Logger.setFileLogger(new FileLogger("LightSwitchAppAsync2.log"));
		
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

		Logger.log("Sleeping for 2s before starting (allows incoming sensor values to commence)...");
		ThreadUtils.delay(2000);
		node.executeCancellableLoop(new CancellableLoop() {
			
			@Override
			protected void setup() {
			}

			@Override
			protected void loop() {
				LightSwitchCmd cmd = publisher.newMessage();
				long startTime, endTime;
				
				Logger.log("Asserting that the room is bright...");
				CPSAssertion a1 = brace.assertAsync(predicateBright, 
						MAX_DELTA, MAX_LATENCY, false);
				
				Logger.log("Turning light off...\n");
				cmd.setCmd((byte)0);
				startTime = System.nanoTime();
				publisher.publish(cmd);
				endTime = System.nanoTime();
				Logger.log("Light should be off, latency = " + (endTime - startTime) + "\n");

				Logger.log("Pausing for " + ACTUATION_PAUSE_TIME + "ms to allow actuation to take effect...\n");
				ThreadUtils.delay(ACTUATION_PAUSE_TIME);
				
				Logger.log("Asserting that the room is dim...");
				CPSAssertion a2 = brace.assertAsync(predicateDim, MAX_DELTA, MAX_LATENCY, false);
				
				Logger.log("Waiting 4s...\n");
				ThreadUtils.delay(4000);
				
				Logger.log("Turning light on...\n");
				cmd.setCmd((byte)1);
				startTime = System.nanoTime();
				publisher.publish(cmd);
				endTime = System.nanoTime();
				Logger.log("Light should be on, latency = " + (endTime - startTime) + "\n");

				Logger.log("Pausing for " + ACTUATION_PAUSE_TIME + "ms to allow actuation to take effect...\n");
				ThreadUtils.delay(ACTUATION_PAUSE_TIME);
				Logger.log("Asserting that the room is bright...");
				CPSAssertion a3 = brace.assertAsync(predicateBright, MAX_DELTA, MAX_LATENCY, false);
				
				Logger.log("Pausing for " + FINAL_PAUSE_TIME + "ms to allow last assertion to run.");
				ThreadUtils.delay(FINAL_PAUSE_TIME);
				
				Logger.log("Were the assertions evaluated? a1 = " + a1.isEvaluated() 
						+ ", a2 = " + a2.isEvaluated() + ", a3 = " + a3.isEvaluated());
				
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