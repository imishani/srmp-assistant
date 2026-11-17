# SRMP for Claude Code

A Claude Code plugin for **[SRMP](https://srmp.readthedocs.io)**, a search-based motion
planning library for robot manipulators, from single arms to dozens of robots at once.

With this plugin installed, Claude can:

- **write SRMP code** using a skill of verified conventions and examples. For instance, it
  knows that `Pose.q` is `[w, x, y, z]` and that `make_planner` values are strings. Every
  sample in the skill is run in SRMP's test suite.
- **run plans for you** through the bundled MCP server (`srmp mcp`). Claude loads robots,
  builds a scene, plans, checks the result, and opens a 3D viewer, all in one persistent
  session.

## Install

The MCP server runs the `srmp` command, so install SRMP first. The server needs Python 3.10
or newer.

```bash
pip install "srmp[mcp]"
```

Then, in Claude Code:

```
/plugin marketplace add imishani/srmp-assistant
/plugin install srmp@srmp
```

Start a new session and ask something like *"Load a Panda, put a 10 cm box at
(0.5, 0.2, 0.4), plan to (0.64, -0.07, 0.5) with the gripper pointing down, and show it in
the viewer."*

## Other assistants

Codex, Gemini CLI, Antigravity and Cursor users don't need this repository. Install the same
skill and MCP server from the `srmp` package:

```bash
pip install "srmp[mcp]"
srmp skill install --for codex      # or gemini, antigravity, cursor, claude, all
srmp mcp install --for codex        # prints the config to add
```

## Documentation

- [Using SRMP with coding assistants](https://srmp.readthedocs.io/en/latest/user_guide/coding_assistants.html)
- [SRMP documentation](https://srmp.readthedocs.io)
- Questions: [Discord](https://discord.gg/JwAWmqE6N)

## About this repository

Releases of SRMP publish this repository automatically, and each release replaces its
contents. Please report issues on [Discord](https://discord.gg/JwAWmqE6N) rather than with
pull requests here.

License: AGPL-3.0-or-later.
