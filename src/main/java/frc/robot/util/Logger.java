package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class Logger {

    public static void init() {
        Work.setupWorkLoglist();
        Comp.setupCompLoglist();
    } 

    public static class Work {

        private static HashMap<String, Object> workList = new HashMap<String, Object>();
        private static HashMap<String, NetworkTableEntry> workEntryList = new HashMap<String, NetworkTableEntry>();
        private static ShuffleboardTab workTab = Shuffleboard.getTab("Work");

        public static void setupWorkLoglist() {
            workList.forEach((a, b) -> {
                workEntryList.put(a, workTab.add(a, b).getEntry());
            });
        }

        public static void post(String key, Object b) {
            // if (b != null) {
                if (workList.containsKey(key)) {
                    workEntryList.get(key).setValue(b);
                } else {
                    workList.put(key, b);                
                }
            // }
        }

        public static void postComplex(String key, Sendable b, WidgetType type) {
            if (!workEntryList.containsKey(key)) {
                workEntryList.put(key, NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Work").getEntry(key));
                workTab.add(key, b).withWidget(type);
            }
        }
    }

    public static class Comp {

        private static HashMap<String, Object> compList = new HashMap<String, Object>();
        private static HashMap<String, NetworkTableEntry> compEntryList = new HashMap<String, NetworkTableEntry>();
        private static ShuffleboardTab compTab = Shuffleboard.getTab("Comp");

        public static void setupCompLoglist() {
            compList.forEach((a, b) -> {
                compEntryList.put(a, compTab.add(a, b).getEntry());
            });
        }

        public static void post(String key, Object b) {
            if (compList.containsKey(key)) {
                compEntryList.get(key).setValue(b);
            } else {
                compList.put(key, b);                
            }

        }

        public static void postComplex(String key, Sendable b, WidgetType type) {
            if (!compEntryList.containsKey(key)) {
                compEntryList.put(key, NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Comp").getEntry(key));
                compTab.add(key, b).withWidget(type);
            }
        }
    }
}
