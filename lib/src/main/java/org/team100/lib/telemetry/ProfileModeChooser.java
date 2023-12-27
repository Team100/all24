package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;

import org.team100.lib.profile.ChoosableProfile;

public class ProfileModeChooser extends NamedChooser<ChoosableProfile.Mode> {
    private static final Map<String, ProfileModeChooser> profileModes = new HashMap<>();

    private ProfileModeChooser(String name) {
        super(name);
    }

    public static ProfileModeChooser get(String name) {
        if (profileModes.containsKey(name)) {
            return profileModes.get(name);
        }
        ProfileModeChooser c = new ProfileModeChooser(name);
        profileModes.put(name, c);
        return c;
    }
}
