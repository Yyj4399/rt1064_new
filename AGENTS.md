# Repository Guidelines

## Project Structure & Module Organization
- `rt1064/project/code`: Custom algorithms (PID, motor, image, planning). Keep files flat here as noted in `本文件夹作用.txt`; add matching headers for new modules.
- `rt1064/project/user/src` and `inc`: Application entry (`main.c`), ISRs, and user-facing hooks.
- `rt1064/project/mdk` and `rt1064/project/iar`: Keil uVision (`rt1064.uvprojx`) and IAR (`rt1064.eww`) projects plus scatter/ICF files and cleanup scripts. Use these to build/flash.
- `rt1064/libraries`: Vendor/Seekfree drivers (`zf_common`, `zf_driver`, `zf_device`), NXP SDK, and docs. Treat as third-party; avoid local edits unless fixing upstream issues.

## Build, Test, and Development
- Keil MDK (5.33+): Open `rt1064/project/mdk/rt1064.uvprojx`, select target `nor_sdram_zf_dtcm`, verify defined symbols and scatter file match `libraries/doc/read me.txt`, then Rebuild + Download to flash over J-Link. Run `rt1064/project/mdk/MDK删除临时文件.bat` to clean artifacts.
- IAR (8.32.4+): Open `rt1064/project/iar/rt1064.eww`, use target `nor_sdram_zf_dtcm`, keep defined symbols and `MIMXRT1064xxxxx_flexspi_nor.icf` linker script aligned, then Download and Debug. `rt1064/project/iar/IAR删除临时文件.bat` cleans temp files.
- Runtime checks: Connect RT1064 board with IPS200 display; monitor UART2 (115200) for boot logs. Ensure the camera init loop exits and menu renders before running motion tests.

## Coding Style & Naming
- C code uses `lower_snake_case` for functions/vars (e.g., `menu_init`, `pit_flag_clear`); macros/constants uppercase; types follow existing patterns (`tagPID_T`, `PIDInitStruct`).
- Prefer 4-space indentation; follow current brace style. Keep headers colocated with sources and include via `zf_common_headfile.h`.
- Add new user modules to `project/code` without nested folders; keep hardware abstraction changes inside `libraries` minimal and documented.

## Testing Guidelines
- No automated tests today; validate on hardware after each change. Rebuild, flash, observe IPS200 output, and watch serial logs for PIT/CSI/UART interrupt noise.
- For motion/PID updates, log encoder data and speed targets; for image or color detection changes, verify frame capture stability.

## Commit & Pull Request Guidelines
- History lacks a formal convention; use short, imperative titles (e.g., "Refine PID gains for corners").
- PRs should describe the feature/fix, hardware used, manual test notes, and attach screenshots or serial snippets when touching display or comms. Link related tasks/issues.
- Avoid committing new binaries/logs from `project/mdk/Objects` or IDE output unless explicitly required; focus on source and config updates.

## Security & Configuration Tips
- Preserve required build symbols (CPU_MIMXRT1064DVL6A, XIP flags, PRINTF/SCANF options) listed in `libraries/doc/read me.txt` when duplicating targets.
- Core init depends on `SYSTEM_CLOCK_600M`, IPS200 orientation (`ips200_set_dir`), and flash settings; document any frequency or memory-map changes.
