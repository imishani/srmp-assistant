# SRMP plugin for Claude Code

Bundles the SRMP motion-planning skill and the `srmp mcp` server.

## Prerequisite

The MCP server runs the `srmp` command, so install it first (Python >= 3.10):

```bash
pip install "srmp[mcp]"
```

## Install

```
/plugin marketplace add imishani/srmp-assistant
/plugin install srmp@srmp
```

## Notes

The skill also works without the MCP server: if `srmp` is not installed the
MCP server simply fails to start, and the skill's instructions and scripts remain usable.

## Other tools

Codex, Gemini, Antigravity and Cursor users: install the skill and MCP server with

```bash
srmp skill install --for <tool>
srmp mcp install --for <tool>
```
