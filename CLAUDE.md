# Nucleo411/ — Binding rules for Claude

This folder is the Nucleo411 board project (STM32F411, STM32CubeIDE).
Authoritative project-wide rules live at the top-level `CLAUDE.md` and at
`Robot/CLAUDE.md`. Read those first; the rules below are the Nucleo411-
specific reinforcements.

## Sibling-ignorance

Nucleo411 must not reach into any other board folder. Do not include from,
reference paths inside, or copy-paste out of `Nano/`, `Discovery/`,
`Nucleo446/`, `PNucleo/`, or `TIVA/`. If another board has a feature
Nucleo411 also wants, factor it into `Robot/` and compose it here — never
cross-link.

## Compose from Robot, don't duplicate it

Shared features live in `Robot/`. Nucleo411 adopts them by:

1. Adding `../Robot/` as a linked source folder in STM32CubeIDE (one-time),
   and adding the relevant Robot sources/includes to the build.
2. Calling `feature_init()` (and any hook registrations) from the startup
   path.
3. Supplying hardware shims the feature requests via its `feature_bind_hw()`
   API — shims live **here**, in Nucleo411.

Canary is already adopted here — use it as the reference for how new Robot
features plug in.

## What belongs in Nucleo411/

- `Core/`, `Drivers/`, STM32CubeIDE artifacts (`.ioc`, `.launch`, linker
  scripts).
- Board-specific CLI words (anything that reads STM32F411 registers,
  dumps HAL state, etc.). Register with `cli_register(...)` from a
  Nucleo411 init function.
- The Nucleo411 `main` that wires Robot features together.
- Limb-sensing code that is currently Nucleo411-specific; if it becomes
  generic, factor it up into `Robot/`.

## What does NOT belong in Nucleo411/

- Board-agnostic protocol code or diagnostics. Those belong in `Robot/`.
- Anything another board would need to copy to use.

## When adding a feature

If it could run on any other board with hardware glue swapped out, factor
it into `Robot/` first, then adopt it here. Do not implement as a
Nucleo411-local module that later has to be generalized.
