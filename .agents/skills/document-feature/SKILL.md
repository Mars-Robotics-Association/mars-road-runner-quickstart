---
name: document-feature
description: Write or update a markdown doc for a robot feature, subsystem, or algorithm. Use when the user asks to document, write docs for, or explain a feature in a .md file.
---

Write a markdown document for the specified feature and save it under `TeamCode/docs/` (create the
file if it doesn't exist; update it in place if it does).

If the document contains formulas, also apply the math-in-markdown conventions.

## Content guidelines

**Describe behavior and purpose, not implementation details.**
- State what the subsystem achieves and why, not how it is currently wired internally.
  *Bad:* "Transitions from MT1 to MT2 after accumulating quality readings."
  *Good:* "Switches to the high-confidence estimate once the sensor has had time to converge."
- Do not describe specific button assignments or control-flow details. Reference the driver guide
  instead.

**What is safe to be concrete about:**
- Field geometry (zone vertices, goal positions) and physics constants (gravity, ceiling height).
- Formulas and algorithms — describe the math, not the line of code that implements it.

## What to omit (avoids doc rot)

- **Line numbers.** `Launcher.java:226` shifts with every edit. Name the method or class instead.
- **Config default values.** Values like `f0 = 12` or `ballExitDelay = 300 ms` change during
  tuning without anyone updating the doc. Describe the *meaning* and *role* of a parameter; leave
  the current value to the code or FTC Dashboard.
- **Internal state-machine details.** The high-level behavior is stable; the exact strategy is not.
