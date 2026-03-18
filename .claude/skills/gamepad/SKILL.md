---
name: gamepad
description: FTC Gamepad input handling and edge detection. Use when reading gamepad input or detecting button press/release events.
---

SDK 11.0 added rising/falling edge detection directly on the `Gamepad` object, e.g.
`gamepad1.leftBumperWasPressed()` and `gamepad1.leftBumperWasReleased()`. SDK 11.1 extended this
to triggers. See the `ConceptGamepadEdgeDetection` sample for usage. Code targeting older SDK
versions must track previous button state manually.

**Level reads vs. edge detection — choose deliberately:**

The SDK uses two different naming conventions, which serve as a built-in signal:
- **`snake_case` fields** (`gamepad1.right_bumper`) — level reads, true the entire time the button is held.
- **`camelCase` methods** (`gamepad1.rightBumperWasPressed()`) — edge detection, true only on the loop where the button was first pressed.

Use level reads when the action is continuous while held (driving, holding a position). Use edge
detection for discrete state changes — toggling a boolean, triggering a one-shot action, setting
a mode.

When two separate buttons set the same boolean in opposite directions via level reads, holding
both simultaneously creates ambiguous priority (last assignment wins). If the intent is "this
button starts, that button stops," use `WasPressed()` on each. A single button used as
push-to-talk (`spinning = gamepad1.right_bumper`) is fine as a level read.
