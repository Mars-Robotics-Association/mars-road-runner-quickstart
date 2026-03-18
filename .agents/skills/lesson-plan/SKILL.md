---
name: lesson-plan
description: Write a lesson plan for a STEM concept in the context of this FTC codebase, targeting students in grades 6–12. Use when the user asks to create a lesson, write a lesson plan, or teach a concept.
---

Write a lesson plan document and save it to `docs/lessons/{concept-slug}.md`
(create the directory if it doesn't exist; overwrite the file if it does).

The lesson must connect the concept directly to real code in this repo. Read the relevant source
files before writing — do not describe code you haven't read.

---

## Step 1 — Understand the request

Identify:
- **The concept** (e.g., "PID control", "coordinate systems", "Kalman filters", "mecanum kinematics")
- **Grade band**: if the user specifies, use it. Otherwise default to writing for both bands:
  - **Middle (6–8):** intuition and analogy first, light math, visible cause-and-effect
  - **Upper (9–12):** include the math, connect to physics/algebra, deeper code reading
- **Slug**: derive a short, lowercase, hyphenated filename from the concept (e.g., `pid-control`, `coordinate-systems`)

---

## Step 2 — Read relevant source files

Search for and read the files in `TeamCode/` most relevant to the concept. Prefer:
- `robot/` — motor controllers, sensors, subsystems
- `opmodes/` — autonomous and teleop programs that *use* those subsystems
- `MecanumDrive.java`, `TankDrive.java` — kinematics and drive control
- `ThreeDeadWheelLocalizer.java`, `PinpointLocalizer.java` — odometry
- `utils/models/` — motor physics models

Reference real class names, method names, and field names from what you actually read.
Do not invent or approximate.

---

## Step 3 — Write the lesson

Use the template below. All sections are required; omit none.
Replace SVG placeholders with actual inline SVG code for any visual that would genuinely
help a student grasp the concept. Skip or merge diagram sections where a diagram would be redundant.

### Template

```markdown
# Lesson: {Full Concept Name}

**Grade band:** Grades {X–Y}
**Prerequisite concepts:** {comma-separated list, or "none"}
**Estimated time:** {N} minutes

---

## What You'll Learn

- {Objective 1 — start each with an action verb: Explain / Calculate / Identify / Describe / Predict}
- {Objective 2}
- {Objective 3}

---

## The Big Idea

{2–4 paragraph plain-language explanation of the concept.
Start with a real-world analogy a 6th grader can picture.
Introduce terms one at a time and immediately anchor each with an example.
For upper grades, add a paragraph that introduces the relevant math or formula.}

---

## Diagram

{Reference one or more SVG diagrams saved as separate files. Guidelines:
- **Save SVGs as separate files** in `docs/lessons/img/`, never inline in the markdown.
  Inline SVG is stripped by GitHub's markdown renderer and won't display.
  Reference them with `![Alt text](img/filename.svg)`.
- 600×300 px viewBox is a good default. Scale as needed.
- Use a light background (#f8f8f8) or white.
- Label all important parts with <text> elements; use a readable sans-serif font.
- Use arrows (<marker> + <line> or <path>) to show direction or flow.
- Favor block diagrams (for control loops), coordinate-axis drawings (for geometry),
  or annotated mechanical diagrams (for motors).
- Keep color minimal: 2–3 colors max. Use blue (#2266cc) for signal/data,
  orange (#e07000) for reference/setpoint, green (#228822) for measured/feedback.
- If no diagram would genuinely help, write a brief descriptive caption instead.}

---

## How This Appears in Our Robot Code

Introduce the specific files and classes, then show short snippets with explanation.

### {ClassName or subsystem name}

> File: `TeamCode/.../FileName.java`

{1–2 sentences on what this class does and why it's relevant.}

```java
// {Snippet — 5–15 lines max. Include just enough context.
//  Use real code copied from the file, not paraphrased.}
```

{Explain what the snippet does, line by line if needed. Connect each part back to the concept.}

{Repeat the file/snippet/explanation block for additional files if relevant.}

---

## Activities

### Activity 1 — {Short title}

{Hands-on activity: predicting behavior, changing a parameter in FTC Dashboard and observing the
result, tracing data flow through code, or drawing a diagram on paper.
Write clear numbered steps. No more than 10 steps. Specify what students observe or record.}

### Activity 2 — {Short title}

{A second activity that goes deeper or approaches the concept differently.
For upper grades, this could be a calculation, a prediction + test, or a code modification exercise.}

---

## Discussion Questions

1. {Question that tests conceptual understanding — answer requires the big idea, not a lookup}
2. {Question that connects to real robot behavior — "what would happen if…"}
3. {Question for upper grades: connects to math or a design tradeoff}

---

## Extension (for fast finishers or advanced students)

{1–3 sentences describing a deeper challenge: a derivation, reading a related paper or doc,
comparing two implementations in the repo (e.g. VelocityMotorPF vs FlywheelStateSpace),
or measuring something on the physical robot.}
```

---

## Flowchart format

Flowcharts and block diagrams (e.g., control loop diagrams) must be saved as **`.drawio.svg`** files,
not hand-written SVG. Create them in draw.io and export as `.drawio.svg` so they remain editable.
The hand-written SVG guidelines below apply only to non-flowchart visuals (charts, coordinate
diagrams, annotated plots, etc.).

---

## SVG guidelines (expanded)

### Control loop block diagram

For any feedback control lesson (PID, LQR, feedforward), draw a standard block diagram:

- Boxes for: Setpoint, Σ (sum/error), Controller, Plant/Motor, Sensor/Encoder
- Arrow from Setpoint into Σ (labeled "r")
- Arrow from Σ to Controller (labeled "e = error")
- Arrow from Controller to Plant (labeled "u = command")
- Arrow from Plant output looping back to Σ (labeled "y = measurement"), with negative sign at Σ
- Color code: setpoint in orange, feedback path in green, forward path in blue

### Coordinate system diagram

For any odometry/pose lesson, draw:
- A rectangle representing the FTC field
- X-axis (pointing right, blue) and Y-axis (pointing up, blue)
- A robot shape (small rectangle) with a heading arrow (orange)
- Labeled tick marks at the 1-tile boundaries

### Velocity / motion profile diagram

For any motor control or trajectory lesson, draw:
- A time-axis (x) and velocity-axis (y) graph
- Show: ramp-up phase (increasing slope), constant velocity, ramp-down
- Label: "acceleration phase", "constant speed", "deceleration"
- If jerk-limited: show the S-curve shape

### SVG arrow marker colors

SVG `<marker>` elements use the fill color from their own `<path>` definition — they do **not**
inherit the stroke color of the parent `<line>`. To draw arrows in different colors, define a
separate `<marker>` element for each color and reference it by ID:

```svg
<marker id="arrG" markerWidth="8" markerHeight="6" refX="7" refY="3" orient="auto">
  <path d="M0,0 L8,3 L0,6 Z" fill="#228822"/>
</marker>
```

Then on each line: `marker-end="url(#arrG)"`.

Use `refX="7"` with `markerWidth="8"` so the arrowhead tip lands exactly at the line endpoint
(not 7 px short of it). Getting this wrong shifts every arrowhead off its target box.

---

## Companion test harness

When a lesson involves a tunable subsystem, offer to create a companion `@TeleOp` OpMode
alongside the lesson document. It gives students a live instrument — they change parameters in
FTC Dashboard and observe effects without writing code. Keep the harness minimal: one toggle
input, the subsystem's own `writeTelemetry()` call, and `@Config` for the target setpoint.

---

## Pedagogical conventions

- **Plain language first, then precise language.** Introduce colloquial descriptions before
  technical terms. When a technical term first appears, bold it and immediately define it.
- **Concrete → abstract.** Always open a section with an observable example before generalizing.
- **No jargon without a definition.** Terms like "state", "gain", "covariance", "feedforward"
  must be defined the first time they appear.
- **Grade calibration:**
  - Grades 6–8: analogies dominate; math limited to ratios, proportions, and basic algebra.
  - Grades 9–12: include equations using standard math notation (rendered as code blocks in
    markdown since this file may be viewed in a plain viewer); connect to physics (F=ma, etc.).
- **Code reading is a skill.** When pointing students at code, give them a specific question
  to answer by reading it, not just "go look at this file."

---

## What to omit

- Line numbers (shift with every edit — reference method/class names instead)
- Config default values (change during tuning; describe their *role*, not their current value)
- Internal state-machine implementation details
- Any code you haven't actually read in Step 2
