import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Main {
	
	
	public static void main(String[] args) {
		
		double kP = 10;
		double kV = 0;
		double kG = -.1;
		
		double kDt = 0.05;
		
		double currentTime = 0;
		int seg = 0;
		
		DriveTrain drive = new DriveTrain(new DriveTrainGearbox(0, 0, kDt), new DriveTrainGearbox(0, 0, kDt), .066, kDt);
		
		Trajectory path = new Path().getTrajectory();
		
		double lpos = drive.getPos_();
		System.out.println("Position\t" + "Goal\t"+ "angle\t"+ "angle Goal\t"+ "left\t"+ "right\t");
		
		 ArrayList<double[]> output = new ArrayList<>();
		
		while (seg <= path.length()-1) {
			
			int seglook = 0;
			if (seg + 6 <= path.length()-1) {
				seglook = seg + 6;
			}else {
				seglook = path.length()-1;
			}
			
			
			double gAngle = (Math.atan((path.get(seglook).y - drive.getPosy_())/ (path.get(seglook).x - drive.getPosy_())));
			double gPos = path.get(seg).position;
			double gVel = path.get(seg).velocity;
			
			double voltage = kP * (gPos - drive.pos_) + kV * (gVel - (lpos - drive.getPos_())/kDt ) ;
			double turn = kG * Pathfinder.boundHalfDegrees((Pathfinder.r2d(Math.PI-gAngle) - Pathfinder.r2d(drive.angle_)));
			
			drive.left.calculate(voltage+turn);
			drive.right.calculate(voltage-turn);
			
			drive.calculate();
			lpos = drive.getPos_();
			System.out.println(drive.getPos_() + "\t" + gPos+ "\t" + Pathfinder.r2d(drive.getAngle_())+ "\t" + Pathfinder.r2d(Math.PI-gAngle)+ "\t" + drive.left.getPosition_()+ "\t" + drive.right.getPosition_());			
			output.add(new double[]{drive.getPosx_(),drive.getPosy_(), path.get(seg).x, path.get(seg).y});
			currentTime++;
			seg++;
		}
		
		double[][] outputAsArray = new double[output.size()][output.get(0).length];

        for(int i = 0; i < output.size(); i++){
        	outputAsArray[i] = output.get(i);
        }

        try {
            CSVHelper.writeCSV(outputAsArray, new File("csv/output.csv"));
        } catch (IOException e){
            System.out.println("!!! Main Could Not Write CSV File !!!");
        }
		
		
				
	}
}
