
public class PurePursuit {
	public static double getHeading(double px, double py, double rx, double ry) {
		return Math.atan2((py - ry), (px - rx));
	}
	
	
}
