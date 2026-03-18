## Workflow Preferences

- Do not run builds or git commands by default — the user handles build testing and git operations themselves. Running builds is fine if the user
  reports a build problem and needs help diagnosing it.
- When the user pastes a URL into a prompt, fetch it with WebFetch before doing anything else. A pasted link is a strong signal that the
  content at that URL is central to what they're asking.
