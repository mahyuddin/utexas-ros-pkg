package edu.utexas.ece.pharos.brace;

public interface CPSPredicate {

	/**
	 * Evaluate at the current time.
	 * 
	 * @param targetTime The ideal time in which the predicate 
	 * should be evaluated.  This can be in the past.
	 * @param delta The maximum difference between the targetTime
	 * and the actual time of evaluation.
	 * @return true if the assertion is true.
	 */
	public boolean evaluate(long targetTime, long delta);
}
