# CM7_1 SRAM reservation fix

## Change

`project/iar/icf/linker_directives_tviibh.icf` now adds `8K` to
`_base_SRAM_CM7_1` and subtracts `8K` from `_size_SRAM_CM7_1`.

This leaves the shared window `0x28080000-0x28081FFF` outside CM7_1 private
SRAM placement. The resulting CM7_1 private SRAM start is `0x28082000`.

## Static evidence

PowerShell arithmetic check:

```powershell
$base = 0x28000000 + (128 * 1KB) + ((512 - 128) * 1KB) + (8 * 1KB)
$base.ToString('X8')
```

Output: `28082000`.

Source check:

```powershell
rg -n "_base_SRAM_CM7_1|_size_SRAM_CM7_1" project/iar/icf/linker_directives_tviibh.icf
```

The two definitions contain `+ 8K` and `- 8K`, respectively. `git diff
--check` completed without errors. No IAR build was run; no files under
`libraries` were changed.
