package edu.utexas.ece.pharos.utils;

public class ThreadUtils {
	public static void delay(long duration) {
		try { 
			Thread.sleep(duration); 
		} catch(InterruptedException ie) { 
			ie.printStackTrace(); 
			Logger.logErr("Sleep interrupted.");
		}
	}
}
