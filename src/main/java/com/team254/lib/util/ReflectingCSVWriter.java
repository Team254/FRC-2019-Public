package com.team254.lib.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file
 */
public class ReflectingCSVWriter<T> {
    private ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
    private PrintWriter mOutput = null;
    private Field[] mFields;

    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        mFields = typeClass.getFields();
        try {
            mOutput = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        // Write field names.
        StringBuilder line = new StringBuilder();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            line.append(field.getName());
        }
        writeLine(line.toString());
    }

    public void add(T value) {
        StringBuilder line = new StringBuilder();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            try {
                if (CSVWritable.class.isAssignableFrom(field.getType())) {
                    line.append(((CSVWritable) field.get(value)).toCSV());
                } else {
                    line.append(field.get(value).toString());
                }
            } catch (IllegalArgumentException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        mLinesToWrite.add(line.toString());
    }

    protected synchronized void writeLine(String line) {
        if (mOutput != null) {
            mOutput.println(line);
        }
    }

    // Call this periodically from any thread to write to disk.
    public void write() {
        while (true) {
            String val = mLinesToWrite.pollFirst();
            if (val == null) {
                break;
            }
            writeLine(val);
        }
    }

    public synchronized void flush() {
        if (mOutput != null) {
            write();
            mOutput.flush();
        }
    }
}
