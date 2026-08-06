---
name: stm32-build
description: "Build and diagnose STM32 projects with the STMicroelectronics STM32Cube VS Code extension toolchain and each project's user-defined CMake configuration. Use whenever the user asks to compile or build, including Chinese requests such as 编译, 帮我编译一下, 重新编译, 使用STM32插件编译, or asks to inspect build errors. Compilation is pre-approved. Never modify source or project configuration files without explicit authorization, and do not use this skill for flashing or debugging."
---

# STM32 Project Build

Build the current STM32 workspace with its STM32Cube-bundled CMake, build tool, compiler, and user-selected CMake preset. Keep source and project configuration files unchanged.

## Rules

- Treat a matching request as authorization to compile and update generated build outputs. Do not ask for compilation permission again.
- Never edit, create, delete, format, regenerate, or clean source and project configuration files without separate explicit approval.
- Do not flash, debug, or run firmware unless explicitly requested.
- Read and follow the nearest `AGENTS.md` before building or diagnosing code.
- Never fall back to a system CMake, Ninja, GCC, or Clang toolchain.

## Build

Run the bundled script from the workspace root and pass the actual workspace path explicitly:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "<skill-directory>\scripts\build_stm32.ps1" -ProjectRoot "<workspace-root>"
```

When the user names a preset or configuration, pass it explicitly:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "<skill-directory>\scripts\build_stm32.ps1" -ProjectRoot "<workspace-root>" -Preset "<build-preset>"
```

If the project uses a build directory without presets, pass `-BuildDirectory`. Pass `-Configuration` only for a requested multi-config configuration.

Without an explicit preset or build directory, let the script select the user's configuration in this order:

1. `STM32_BUILD_PRESET` environment variable.
2. VS Code `cmake.buildPreset` workspace setting.
3. First visible build preset declared by `CMakeUserPresets.json` or its includes.
4. First visible build preset declared by `CMakePresets.json` or its includes.
5. Most recently updated existing `CMakeCache.txt` when no build preset exists.

The script invokes `cmake --build --preset <name>` for presets, preserving custom targets, configuration, jobs, environment, and other preset fields. It validates that the selected cache's CMake, build tool, and C compiler are all from the same STM32Cube bundle before building.

If selection or validation fails, report the available presets and the exact configuration problem. Do not choose another toolchain or edit the presets.

## Report

Capture the full output and exit code. On success, report the selected configuration source, preset or build directory, output `.elf` path when present, memory usage, and warnings.

On failure, inspect relevant files read-only and identify the first root cause before cascaded errors. For each actionable issue report:

1. Exact file and line, plus the related function, symbol, preset, or command.
2. Cause, build-stage impact, and likely runtime impact.
3. A concrete fix or optimization recommendation without applying it.

Separate compiler, assembler, linker, CMake, toolchain, and generated-file problems. Report relevant warnings independently.

## Timing

When diagnosing source, inspect execution order, blocking behavior, callback or interrupt context, counter rollover, and delays. In projects whose `AGENTS.md` defines `HAL_Delay(n)` as `(n + 1) ms`, treat `HAL_Delay(0)` as a `1 ms` delay.

## Probe

Use `-ProbeOnly` to validate selection without compiling:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "<skill-directory>\scripts\build_stm32.ps1" -ProjectRoot "<workspace-root>" -ProbeOnly
```
