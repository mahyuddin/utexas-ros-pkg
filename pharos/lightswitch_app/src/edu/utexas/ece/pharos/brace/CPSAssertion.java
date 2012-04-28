package edu.utexas.ece.pharos.brace;

import edu.utexas.ece.pharos.utils.Logger;

/**
 * A handle to a CPS assertion.
 * 
 * @author Chien-Liang Fok
 */
public class CPSAssertion {

	private CPSPredicate predicate;
	
	private boolean evaluated = false;
	private long initTime, delta, maxLatency;
	private boolean failSilently = false;
	
	/**
	 * Counts the number of times this assertion was 
	 * evaluated.  It is useful for continuous assertions.
	 */
	private int evaluationCount = 0;
	
	/**
	 * The constructor.
	 * 
	 * @param predicate 
	 *   The assertion's predicate, which may reference both logical
	 *   and mapped logical variables.
	 * @param delta 
	 *   The maximum difference between the ideal time stamps
	 *   of the physical variables and their actual time stamps.
	 * @param maxLatency
	 *   The maximum amont of time that can pass before this
	 *   assertion <b>must</b> be evaluated.
	 * @param failSilently
	 *   Whether to abort this assertion silently if maxLatency 
	 *   is not met.
	 */
	public CPSAssertion(CPSPredicate predicate, long delta, long maxLatency, 
			boolean failSilently) 
	{
		this.predicate = predicate;
		this.delta = delta;
		this.maxLatency = maxLatency;
		this.failSilently = failSilently;
		this.initTime = System.currentTimeMillis();
	}
	
	/**
	 * Aborts this assertion.  Prevents it from being evaluated.
	 */
	public synchronized void abort() {
		evaluated = true;
	}
	
	/**
	 * 
	 * @return Whether this assertion was evaluated.
	 */
	public synchronized boolean isEvaluated() {
		return evaluated;
	}
	
	/**
	 * Changes the state of this assertion to be not evaluated.
	 */
	public synchronized void reset() {
		evaluated = false;
	}
	
	/**
	 * 
	 * @return The number of times this assertion was evaluated.
	 */
	public int getEvaluationCount() {
		return evaluationCount;
	}
	
	/**
	 * Evaluate the assertion.  This method exits if the assertion 
	 * evaluates to false.
	 */
	public synchronized void evaluate() {
		if (!evaluated) {
			evaluated = true;
			if (System.currentTimeMillis() - initTime < maxLatency) {
				evaluationCount++;
				if (!predicate.evaluate(initTime, delta)) {
					Logger.logErr("Assertion failed: " + predicate);
					System.exit(-1);
				}
			} else {
				if (!failSilently) {
					Logger.logErr("Assertion failed due to maxLatency violation.");
					System.exit(-1);
				}
			}
		}
	}
}
