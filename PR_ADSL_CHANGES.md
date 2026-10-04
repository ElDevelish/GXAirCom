# PR Details: ADS-L Code + Documentation Changes

This branch (`claude/competent-keller-e004c4`) contains both the OLED display timeout feature (v8.7.1) and a set of ADS‑L related code and documentation changes. This file documents what the PR contains so reviewers can focus on ADS‑L changes in addition to the display work.

## Summary

- Feature: Add air-module OLED display timeout options (Always On / Always Off / 1 / 2 / 5 minutes), persistence, web UI exposure, and wake-on-page-button behavior.
- ADS‑L work: Research, documentation and corresponding code/config changes collected in this branch (not only docs). See full file list below.

## Files changed (high level)

Relevant documentation and ADS‑L research files:
- `ADS-L_IMPLEMENTATIONS_RESEARCH.md`
- `ADS-L_INTEROPERABILITY_ANALYSIS.md`
- `OPENACE_ADSL_DETAILED_ANALYSIS.md`
- `OPENACE_ADSL_QUICK_REFERENCE.md`
- `OPENACE_ANALYSIS_SUMMARY.md`
- related PDFs and comparison documents in repo root

Source/config files changed (code that may affect runtime):
- `platformio.ini` (build/config changes)
- `src/main.cpp` (core application changes and feature wiring)
- `src/main.h` (settings / enums additions)
- `src/enums.h` (new enums)
- `src/fileOps.cpp` (persistence load/save changes)
- `src/WebHelper.cpp` (web API exposure / JSON fields)
- `src/oled.cpp`, `src/oled.h` (OLED timeout/wake logic)
- `src/web/orig/fullsettings.html`, `src/web/website.h` (web UI updates)

Other repo artifacts added/updated:
- `bin/_version.txt`, `bin/version.txt` (version bump and release notes)
- `OLED_DISPLAY_TIMEOUT_CHANGELOG.md` (revert notes)

## Why this matters

- The ADS‑L documents are not just informational; the branch aggregates instrumentation, configuration and code changes intended to support ADS‑L workflows and testing in this repository. Reviewers should inspect both the documentation files and the runtime code changes listed above.

## How to review

1. Compare this branch with `master` on GitHub (PR contains the full diff).
2. Review the ADS‑L documentation files for intended behavior and compatibility notes.
3. Inspect `src/*` runtime changes for API/behavior impacts, especially `platformio.ini` and `src/main.cpp`.
4. Run PlatformIO builds for your target boards to verify no regressions.

## Notes

- This branch also contains binary artifacts under `bin/` (built firmware). Please review whether you want binary files included in the PR or whether they should be removed from the commit and delivered via release artifacts.

---
Commit: branch `claude/competent-keller-e004c4` — intended target: `master`
