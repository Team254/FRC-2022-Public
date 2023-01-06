package com.team254.lib.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.io.FileWriter;
import java.io.IOException;

public class TuningLogger {
    private PrintWriter mWriter = null;

    public TuningLogger(String filePath) {
        try {
            mWriter = new PrintWriter(new FileWriter(filePath));
        } catch (IOException ignored) {

        }
    }

    public void logData(String mapName, double range, double value) {
        if (mWriter != null) {
            String line = mapName + ".put(new InterpolatingDouble(" + range + "), new InterpolatingDouble(" + value + "));\n";
            mWriter.write(line);
            mWriter.flush();
            System.out.println("Wrote: " + line);
        } else {
            System.err.println("No print writer created to write " + mapName);
        }
    }
}
