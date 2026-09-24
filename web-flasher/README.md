# Shift light flasher

A static page that puts the shift light firmware on a board from Chrome or
Edge on a computer, over Web Serial, with Espressif's esptool-js (pinned to
0.7.0 on jsDelivr). No build step: `index.html`, `flasher.js` and `style.css`
are the whole page. Same layout and flash logic as the Touge radio flasher, in
the product page's colours.

## What it does

**Board.** Connect reads the chip and its flash, and picks the build:

| Chip | Flash | PSRAM | Env |
|---|---|---|---|
| ESP32-C3 | 4 MB | none | `esp32-c3` (carrier board, C3 SuperMini) |
| ESP32-S3 | 4 MB | 2 MB | `esp32-s3-zero` (prototype, S3-Zero) |

Anything else gets "not a shift light board, nothing flashed". "I know what
this is" picks by hand; the chip family and flash size are still checked
before writing. While the port is open it also reads the md5 of the partition
table at `0x8000`, so a board that isn't on this build's layout (a new board,
or other firmware) gets Fresh install preselected and Update greyed out.

**Update** writes the app at `0x10000`, then `boot_app0.bin` at `0xe000` so the
bootloader runs app0. It never writes NVS at `0x9000`, which holds the saved
config. It re-checks the partition table md5 before writing and refuses on a
different layout.

**Fresh install** erases the chip and writes bootloader `0x0`, partition table
`0x8000`, otadata `0xe000` and app `0x10000`. The board boots on defaults.

Every file is checked against the manifest's sha256 after download and before
anything is written, and against its md5 on the flash after writing.

**A batch.** After a good flash the dialog offers "Flash the next board". It
asks for the port again (every C3 has its own USB serial number, so Chrome
treats each one as a new device), detects the chip and flashes in the same
mode. The count and each board's MAC go in the serial log.

**Reset and the port.** Reading the chip puts it in the ROM bootloader. The
page resets it back into its firmware and closes the port after the chip is
read, after a flash (good or failed), when the flash dialog closes, and on
leaving the page (never mid-write). The reset is a `CustomReset` with
`D0|R1|W200|R0|W200`: EN pulsed with the boot strap high, on the chip's own
USB-Serial/JTAG. esptool-js 0.7.0's `after("hard_reset")` only drops RTS,
which is already low, so it resets nothing.

## Publishing a build

1. Commit and push the firmware. `publish-release.ps1` refuses uncommitted
   changes under `src`, `include`, `lib` or `platformio.ini`, and records HEAD
   as the build's source link.
2. Run it from PowerShell:

   ```powershell
   .\web-flasher\publish-release.ps1 -Notes "What changed, one line"
   ```

   It runs `pio run` for both envs itself, checks each image's chip id and
   the partition layout, copies the images to `firmware/<build>/<env>/`, adds
   the build to `releases.json` (next number unless `-Build` is given, which
   replaces that build) and copies the page. Default `-Site` is
   `C:\Projects\esp32-shift-light-R53-mini-pages`.
3. Commit and push the site (below).

For a bench test from the working tree, stage anywhere with `-Bench` and serve
it locally. Web Serial works on localhost without HTTPS. A bench build shows a
red banner and no source link.

```powershell
.\web-flasher\publish-release.ps1 -Bench -Site C:\tmp\flasher
cd C:\tmp\flasher; python -m http.server 8000
```

## Hosting on GitHub Pages

The images are served from the Pages site itself, not from Release assets: a
release download doesn't send CORS headers, so the page can't `fetch` it
(tested on the Touge flasher, 2026-09-23).

The site lives on an orphan `gh-pages` branch so the binaries stay out of
`main`:

```powershell
cd C:\Projects\esp32-shift-light-R53-mini
git worktree add --orphan -b gh-pages ..\esp32-shift-light-R53-mini-pages
.\web-flasher\publish-release.ps1 -Notes "..."
cd ..\esp32-shift-light-R53-mini-pages
git add -A
git commit -m "Build <n>"
git push -u origin gh-pages
```

Then on GitHub: Settings, Pages, "Deploy from a branch", `gh-pages`, `/ (root)`.
The page is at https://mrblahhhh.github.io/esp32-shift-light-R53-mini/.

Each build adds about 1.5 MB (two boards, about 0.7 MB of app each). To drop
an old build, delete `firmware/<n>/` and its entry in `releases.json`.

## releases.json

```json
{
  "schema": 1,
  "sourceRepo": "https://github.com/MrBlahhhh/esp32-shift-light-R53-mini",
  "releases": [{
    "build": 1,
    "date": "2026-09-24",
    "bench": false,
    "notes": "",
    "sourceCommit": "<commit>",
    "boards": {
      "esp32-c3": {
        "partitionTable": { "offset": "0x8000", "size": 3072, "md5": "..." },
        "partitions": { "nvs": "0x9000", "otadata": "0xe000", "app0": "0x10000", "app1": "0x1f0000", "spiffs": "0x3d0000", "coredump": "0x3f0000" },
        "update": [
          { "what": "app", "path": "firmware/1/esp32-c3/shiftlight-esp32-c3-build1.bin", "offset": "0x10000", "size": 703712, "md5": "...", "sha256": "..." },
          { "what": "otadata", "path": "firmware/1/esp32-c3/boot_app0.bin", "offset": "0xe000", "size": 8192, "md5": "...", "sha256": "..." }
        ],
        "fresh": [
          { "what": "bootloader", "offset": "0x0", "...": "..." },
          { "what": "partitions", "offset": "0x8000", "...": "..." },
          { "what": "otadata", "offset": "0xe000", "...": "..." },
          { "what": "app", "offset": "0x10000", "...": "..." }
        ]
      }
    }
  }]
}
```

Files are written in list order.
