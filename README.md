# SRMP for AI coding assistants

Use **[SRMP](https://srmp.readthedocs.io)**, a search-based motion planning library for
robot manipulators, from your AI coding assistant. SRMP plans for anything from a single
arm to dozens of robots at once. It works with **Claude Code, OpenAI Codex, Gemini CLI,
Google Antigravity and Cursor**.

SRMP gives your assistant two things:

- **A skill**, so it writes correct SRMP code. The skill holds verified conventions and
  examples. For instance, `Pose.q` is `[w, x, y, z]`, and `make_planner` values are
  strings. Every sample in it is run in SRMP's test suite.
- **An MCP server** (`srmp mcp`), so it can run plans for you. The assistant loads robots,
  builds a scene, plans, checks the result and opens a 3D viewer, all in one persistent
  session.

Both come with the `srmp` Python package. The MCP server needs Python 3.10 or newer.

```bash
pip install "srmp[mcp]"
```

## Claude Code

Install both pieces as a plugin from this repository:

```
/plugin marketplace add imishani/srmp-assistant
/plugin install srmp@srmp
```

## Codex, Gemini CLI, Antigravity, Cursor

Use the `srmp` command. Swap in `gemini`, `antigravity` or `cursor` for `codex`, or use
`all` for every assistant:

```bash
srmp skill install --for codex          # copies the skill into the assistant's skills folder
srmp mcp install --for codex            # prints the MCP config to add (edits nothing)
```

`srmp skill install` installs into the current project by default; add `--user` to install
it for your account instead. Run it again after upgrading SRMP.

The same two commands work for Claude Code too, if you'd rather not use the plugin.

## Try it

Start a new session and ask something like *"Load a Panda, put a 10 cm box at
(0.5, 0.2, 0.4), plan to (0.64, -0.07, 0.5) with the gripper pointing down, and show it in
the viewer."*

## Documentation

- [Using SRMP with coding assistants](https://srmp.readthedocs.io/en/latest/user_guide/coding_assistants.html)
- [SRMP documentation](https://srmp.readthedocs.io)
- Questions: [Discord](https://discord.gg/JwAWmqE6N)

## About this repository

Releases of SRMP publish this repository automatically, and each release replaces its
contents. Please report issues on [Discord](https://discord.gg/JwAWmqE6N) rather than with
pull requests here.

License: AGPL-3.0-or-later.
