import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class CSVHelper {

    public static double[][] parseCSV(File file) throws FileNotFoundException {
        List<List<String>> lines = new ArrayList<>();
        Scanner inputStream = new Scanner(file);

        double[][] returnMe;

        while (inputStream.hasNext()) {
            String line = inputStream.next();
            String[] values = line.split(",");

            lines.add(Arrays.asList(values));
        }

        inputStream.close();

        returnMe = new double[lines.size()][lines.get(0).size()];

        int lineNum = 0;
        for (List<String> line : lines) {
            int columnNum = 0;

            for (String val : line) {
                returnMe[lineNum][columnNum] = Double.parseDouble(val);
                columnNum++;
            }

            lineNum++;
        }
        return returnMe;
    }

    public static void writeCSV(double[][] array, File writeTo) throws IOException {
        FileWriter writer = new FileWriter(writeTo);
        String writeMe = "";
        for (double[] vals : array) {
            for (double val : vals) {
                writeMe += Double.toString(val);
                writeMe += ",";
            }
            writeMe += "\n";
        }

        writer.write(writeMe);
        writer.close();
    }
}