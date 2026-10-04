# OLED Display Timeout Revert Notes

This file records the air-module OLED display timeout feature added for the GXAirCom project so it can be reverted cleanly.

## Summary of changes

- Added a new display-timeout enum in `src/enums.h` with the values:
  - `0` = Always On
  - `1` = Always Off
  - `2` = 1 min
  - `3` = 2 min
  - `4` = 5 min
- Added a persisted `displayTimeout` field to `SettingsData` in `src/main.h`.
- Loaded and saved the setting in `src/fileOps.cpp` using the key `DispTmo`.
- Added OLED sleep timer logic and wake-on-button behavior in `src/oled.h` and `src/oled.cpp`.
- Exposed the setting to the web UI in `src/WebHelper.cpp`.
- Added the Display Timeout dropdown to the HTML form in `src/web/orig/fullsettings.html`.

## Files changed

- `src/enums.h`
- `src/main.h`
- `src/oled.h`
- `src/oled.cpp`
- `src/fileOps.cpp`
- `src/WebHelper.cpp`
- `src/web/orig/fullsettings.html`

## Revert procedure

If you want to remove the feature manually, revert the above files to the previous version and remove the `DispTmo` preference from the ESP32 flash settings if it was saved.

Typical Git revert flow:

```bash
git checkout -- src/enums.h src/main.h src/oled.h src/oled.cpp src/fileOps.cpp src/WebHelper.cpp src/web/orig/fullsettings.html
```

If you are not using Git, remove the added `displayTimeout` references and delete the `DispTmo` key from the saved preferences on-device.

## Notes

- The feature only affects the Air Module display path.
- The timeout does not apply to the Ground Station display option path.
- A button press on the page-switch button wakes the OLED from sleep when the timeout is not set to Always Off.

## Release

Released as v8.7.1 on 2026-10-04.

- Version: `v8.7.1`
- Summary: Added air-module OLED display timeout (Always On, Always Off, 1,2,5 minutes), persistence, web UI exposure, and wake-on-page-button behavior. Files changed are listed above.

Commit and push this change to record the release.
