package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.*;

public class DoubleLookupLerp extends LookupLerp<Double, Double> {
    @Override
    protected Double lerp(Double x1, Double y1, Double x2, Double y2, Double key) {
        return y1 + (y2 - y1) * (key - x1) / (x2 - x1);
    }

    public static DoubleLookupLerp loadFromCsv(String path) {
        DoubleLookupLerp dll = new DoubleLookupLerp();
        try (BufferedReader br = new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), path)))) {
            br.lines().forEach(l -> {
                String[] cols = l.split(",");
                dll.put(Double.parseDouble(cols[0]), Double.parseDouble(cols[1]));
            });
        } catch (FileNotFoundException e) {
            System.err.println("File " + path + " in deploy directory was not found. Double check your path.");
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ArrayIndexOutOfBoundsException e) {
            System.err.println("Make sure your CSV has two columns.");
            throw new RuntimeException(e);
        } catch (NumberFormatException e) {
            System.err.println("Make sure your CSV contains only numbers.");
            throw new RuntimeException(e);
        }
        return dll;
    }
}
