package edu.utexas.ece.pharos.utils;

public class ByteUtils {

	  public static int unsignedByteToInt(byte b) {
		  int result = 0;
		  result = 0x00FF & b;
		  return result;
	  }
}
