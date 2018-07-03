import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Main {

	public static void main(String[] args) {

		// Control loops constants
		double kP = 7;
		double kV = 0;
		double kG = -.5;

		double kDt = 0.05;

		// increment for simulation
		int seg = 0;

		// Drivetrain object used for simulation
		DriveTrain drive = new DriveTrain(new DriveTrainGearbox(0.0, 0.0, kDt), new DriveTrainGearbox(0.0, 0.0, kDt),
				.066, kDt);

		// Path for simulation
		Trajectory path = new Path().getTrajectory();

		// data to log
		ArrayList<Object> type = new ArrayList<>();
		ArrayList<Object> x = new ArrayList<>();
		ArrayList<Object> y = new ArrayList<>();
		ArrayList<Object> v = new ArrayList<>();
		ArrayList<Object> t = new ArrayList<>();
		ArrayList<Object> a = new ArrayList<>();
		ArrayList<Object> p = new ArrayList<>();

		// loops through all path segs
		while (seg <= path.length() - 1) {

			// used for angle tracking
			double gAngle;

			// look ahead seg
			int seglook = 0;

			// checks if at the end of path BUGS
			if (seg + 5 < path.length()) {
				seglook = seg + 5;

				// looks ahead and calculates the angle needed to turn to
				gAngle = Pathfinder.boundHalfDegrees(
						Math.atan2((path.get(seglook).y - drive.getPosy_()), (path.get(seglook).x - drive.getPosx_())));
			} else {
				// seglook = path.length()-1;
				gAngle = Pathfinder.boundHalfDegrees(path.get(path.length() - 1).heading);
			}

			// System.out.println(Pathfinder.r2d(gAngle));

			// goal pos and velocity for control loop
			double gPos = path.get(seg).position;
			double gVel = path.get(seg).velocity;

			// Position Loop
			double voltage = kP * (gPos - drive.pos_);

			// Gyro Loop
			double turn = kG * Pathfinder.boundHalfDegrees((Pathfinder.r2d(gAngle) - Pathfinder.r2d(drive.angle_)));

			// Calculates new pos and vel
			drive.left.calculate(Math.min(12.0, Math.max(-12.0, voltage + turn)));
			drive.right.calculate(Math.min(12.0, Math.max(-12.0, voltage - turn)));

			// calculates pos and vel of whole model
			drive.calculate();

			// log all the things
			type.add("path");
			x.add(path.get(seg).x);
			y.add(path.get(seg).y);
			v.add(voltage);
			t.add(seg * kDt);
			a.add(Pathfinder.r2d(path.get(seg).heading));
			p.add(path.get(seg).position);

			type.add("robot");
			x.add(drive.getPosx_());
			y.add(drive.getPosy_());
			v.add(voltage);
			t.add(seg * kDt);
			a.add(Pathfinder.r2d(drive.angle_));
			p.add(drive.getPos_());

			seg++;
		}

		//Logging stuffs
		ArrayList<CSVObject> list = new ArrayList<CSVObject>();

		list.add(new CSVObject("time", t));
		list.add(new CSVObject("type", type));
		list.add(new CSVObject("x", x));
		list.add(new CSVObject("y", y));
		list.add(new CSVObject("voltage", v));
		list.add(new CSVObject("angle", a));
		list.add(new CSVObject("pos", p));

		try {
			CSVHelper.writeCSV(list, new File("csv/output.csv"));
		} catch (IOException e) {
			System.out.println("!!! Main Could Not Write CSV File !!!");
		}

	}
}
