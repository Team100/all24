package org.team100.lib.selftest;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;

/** Collects test results. */
@ExcludeFromJacocoGeneratedReport
public class SelfTestListener {
    private static final String kReset = "\033[0m";
    private static final String kGreen = "\033[1;92m";
    private static final String kRed = "\033[1;91m";
    boolean success = true;
    List<String> outcomes = new ArrayList<>();

    public void fail(Command command, String format, Object... vars) {
        success = false;
        String msg = kRed + command.getName() + " FAILED: " + kReset + String.format(format, vars);
        Util.println(msg);
        outcomes.add(msg);
    }

    public void pass(Command command, String format, Object... vars) {
        String msg = kGreen + command.getName() + " PASSED: " + kReset + String.format(format, vars);
        Util.println(msg);
        outcomes.add(msg);
    }

    public String summary() {
        StringBuilder b = new StringBuilder();
        if (success) {
            pass(b);
        } else {
            fail(b);
        }
        for (String s : outcomes) {
            b.append(s);
            b.append("\n");
        }
        b.append("\n");
        return b.toString();
    }

    private void pass(StringBuilder b) {
        b.append(kGreen);
        b.append("\n");
        b.append("########     ###     ######   ###### \n");
        b.append("##     ##   ## ##   ##    ## ##    ##\n");
        b.append("##     ##  ##   ##  ##       ##      \n");
        b.append("########  ##     ##  ######   ###### \n");
        b.append("##        #########       ##       ##\n");
        b.append("##        ##     ## ##    ## ##    ##\n");
        b.append("##        ##     ##  ######   ###### \n");
        b.append("\n");
        b.append(kReset);
    }

    private void fail(StringBuilder b) {
        b.append(kRed);
        b.append("\n");
        b.append("########    ###    #### ##      \n");
        b.append("##         ## ##    ##  ##      \n");
        b.append("##        ##   ##   ##  ##      \n");
        b.append("######   ##     ##  ##  ##      \n");
        b.append("##       #########  ##  ##      \n");
        b.append("##       ##     ##  ##  ##      \n");
        b.append("##       ##     ## #### ########\n");
        b.append("\n");
        b.append(kReset);
    }
}