import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Main {

	public static void main(String[] args) {

		// Control loops constants
		double kP = 100;
		//double kV = 0;
		double kG = .04;

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
			double gAngle = 0;

			// look ahead seg
			double lookahead = .55;

			int closepoint = 0;
			double closedistance = 10000.0;

			for (int i = 0; i <= path.length() - 1; i++) {
				double distance = Math.hypot(path.get(i).y - drive.getPosy_(), path.get(i).x - drive.getPosx_());
				if (distance < closedistance) {
					closedistance = distance;
					closepoint = i;
				}
			}
			// System.out.println(closedistance);

			int seglook = closepoint;

			// checks if at the end of path BUGS
			boolean foundPoint = false;
			while (!foundPoint) {

				if (seglook <= path.length() - 1) {

					if (Math.hypot(path.get(seglook).y - drive.getPosy_(),
							path.get(seglook).x - drive.getPosx_()) >= lookahead) {
						foundPoint = true;
						gAngle = Math.atan2((path.get(seglook).y - drive.getPosy_()),
								(path.get(seglook).x - drive.getPosx_()));
					} else {
						seglook++;
					}

				} else {
					foundPoint = true;
					seglook = path.length() - 1;
					gAngle = path.get(seglook).heading;
				}

			}
			// System.out.println(Math.hypot(path.get(seglook).y - drive.getPosy_(),
			// path.get(seglook).x - drive.getPosx_()));

			// gAngle = path.get(seg).heading;

			// System.out.println(Pathfinder.r2d(gAngle));

			// goal pos and velocity for control loop
			double gPos = path.get(seg).position;
			double gVel = path.get(seg).velocity;
			double voltage = 0;
			double turn = 0;

			// Position Loop
			voltage = path.get(seg).velocity * 10 / 4.59 + kP * (gPos - drive.pos_);

			// Gyro Loop
			turn = kG * Pathfinder.boundHalfDegrees((Pathfinder.r2d(gAngle) - Pathfinder.r2d(drive.angle_)));

			// Calculates new pos and vel
			drive.left.calculate(Math.min(10.0, Math.max(-10.0, voltage)) - turn);
			drive.right.calculate(Math.min(10.0, Math.max(-10.0, voltage)) + turn);

			// calculates pos and vel of whole model
			drive.calculate();

			// log all the things
			type.add("path");
			x.add(path.get(seg).x);
			y.add(path.get(seg).y);
			v.add(voltage);
			t.add(seg * kDt);
			a.add( Pathfinder.boundHalfDegrees(Pathfinder.r2d(path.get(seg).heading)));
			p.add(path.get(seg).position);

			type.add("robot");
			x.add(drive.getPosx_());
			y.add(drive.getPosy_());
			v.add(voltage);
			t.add(seg * kDt);
			a.add( Pathfinder.boundHalfDegrees(Pathfinder.r2d(drive.angle_)));
			p.add(drive.getPos_());

			seg++;
		}

		// Logging stuffs
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
