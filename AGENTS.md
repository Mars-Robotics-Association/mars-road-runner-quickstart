# AGENTS Notes

Use these helper scripts from repo root:

- `preflight-check.bat` / `preflight-check.sh`: check for dependencies before building. Saves time debugging build problems
- `update-submodules.bat` / `update-submodules.sh`: assists with managing the submodules (pass a submodule name, e.g. `MarsCommonFtc`, or run with no args to pick from a list)

Non-obvious environment caveats:

- In restricted/sandboxed environments, `gradlew`, `preflight-check Y` and Git index writes probably just need elevation. Try that before thrashing on other workarounds.
