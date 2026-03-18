---
name: lessons-learned
description: Capture lessons from the current session into the appropriate skill files. Use when the user asks to record lessons learned, update skills from the session, or capture gotchas discovered during work.
---

Review the session and identify lessons worth preserving — non-obvious behaviors, gotchas, decisions made with reasoning, and corrections to previous assumptions.

## Step 1 — Identify candidate lessons

Look over the session for:
- Surprises or gotchas that weren't anticipated
- Decisions where the reasoning should be preserved (not just the outcome)
- Corrections to something previously assumed or done wrong
- Facts about tools/libraries/the build system that would be easy to re-discover painfully

Skip anything obvious, temporary, or already documented.

## Step 2 — Read existing skills

Read all SKILL.md files under `.claude/skills/` and any referenced docs. For each candidate lesson, determine:

- **Which skill is it relevant to?** Match by topic — build system lessons go to `build`, FTC SDK lessons go to `ftc-robot-code`, etc.
- **Does a relevant skill exist?** If not, consider whether the lesson is narrow enough to hold off, or broad enough to warrant a new skill.

## Step 3 — Decide placement

For each lesson, choose the right home:

| Situation | Placement |
|---|---|
| Short rule or fact, needed as quick context when the skill is active | Add inline to `SKILL.md` |
| Detailed explanation, only needed when working on that specific thing | Create a referenced doc (e.g. `topic.md`) and link from `SKILL.md` |
| Spans multiple skills or is build/project-structure knowledge | `build` skill |
| No existing skill fits | Create a new skill directory only if the lesson is likely to recur |

**Prefer inline** for things that affect day-to-day decisions. **Prefer a referenced doc** when the detail would bloat the skill or is only consulted occasionally.

## Step 4 — Write the updates

Edit the relevant files. For referenced docs:
- Create the file in the same directory as its skill
- Add a one-line link in `SKILL.md` pointing to it with brief context on when to consult it

Keep language specific and grounded — write what actually happened and why it matters, not generic advice.
