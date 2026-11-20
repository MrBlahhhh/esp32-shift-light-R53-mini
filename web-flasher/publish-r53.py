#!/usr/bin/env python3
"""Publish a K-line + CAN bridge and dash screen build to the flasher site.

The C3 board's builds come from publish-release.ps1. The K-line + CAN board
(XIAO ESP32-S3, plain and screen builds) and the dash screens build from the
R53 bridge repo, which is private, so this takes their images from that
checkout instead:

    python3 web-flasher/publish-r53.py --repo ~/work/R53_Mini_Kline_Canbus_Logger_Shiftlight \\
        --site ../esp32-shift-light-R53-mini-pages --notes "What changed, one line"

It runs `pio run` for every env (pass --no-build to use what's built), refuses
uncommitted firmware changes, checks each image is an ESP32-S3 image and each
partition table's layout, copies the images to firmware/<build>/<env>/, adds
the build to releases.json (the next number unless --build is given, which
replaces that build) and copies the page. Then commit and push the site.
"""
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAGE_FILES = ["index.html", "flasher.js", "style.css"]
BRIDGE_DIR = "firmware/esp32_shiftlight_wideband"
SCREEN_DIR = "screen"
BRIDGE_ENVS = ["xiao_esp32s3", "xiao_esp32s3_lcd"]
SCREEN_ENVS = ["screen-lcd21", "screen-lcd28c", "screen-amoled175", "screen-amoled241", "screen-amoled241v1"]
# Source that goes into an image; anything uncommitted here and the build's commit would lie.
SOURCE_PATHS = [f"{BRIDGE_DIR}/src", f"{BRIDGE_DIR}/include", f"{BRIDGE_DIR}/lib", f"{BRIDGE_DIR}/platformio.ini",
                f"{SCREEN_DIR}/src", f"{SCREEN_DIR}/platformio.ini", f"{SCREEN_DIR}/CMakeLists.txt",
                f"{SCREEN_DIR}/partitions.csv", f"{SCREEN_DIR}/sdkconfig.defaults",
                f"{SCREEN_DIR}/sdkconfig.lcd.defaults", f"{SCREEN_DIR}/sdkconfig.amoled.defaults",
                f"{SCREEN_DIR}/bootclip.py", f"{SCREEN_DIR}/boot", "shared"]
ESP32S3_CHIP_ID = 9
APP_DESC_MAGIC = 0xABCD5432
# The screen's boot clip partition (screen 0.1.1 on). bootclip.py writes bootclip.bin into the
# build: "R5CLIP01", the clip's length and the lettering frames, then the MJPEG; a board with no
# clip gets 16 bytes of 0xFF, which the player reads as "no clip" and which clears an old one.
CLIP_MAGIC = b"R5CLIP01"
CLIP_EMPTY = b"\xff" * 16


def fail(text):
    sys.exit(f"publish-r53: {text}")


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout.strip()


def pio():
    for candidate in [shutil.which("pio"), Path.home() / ".platformio/penv/bin/pio"]:
        if candidate and Path(candidate).exists():
            return str(candidate)
    fail("pio not found (PlatformIO)")


def check_image(path, app):
    data = path.read_bytes()
    if data[0] != 0xE9:
        fail(f"{path} is not an ESP image")
    chip = struct.unpack_from("<H", data, 12)[0]
    if chip != ESP32S3_CHIP_ID:
        fail(f"{path} is for chip id {chip}, not the ESP32-S3")
    if not app:
        return None
    at = data.find(struct.pack("<I", APP_DESC_MAGIC))
    if at < 0:
        fail(f"{path} has no app description")
    return data[at + 16:at + 48].split(b"\0")[0].decode()


def read_partitions(path):
    table = {}
    data = path.read_bytes()
    for i in range(0, len(data), 32):
        magic, ptype, subtype, offset, size, label = struct.unpack_from("<HBBII16s", data, i)
        if magic != 0x50AA:
            break
        table[label.split(b"\0")[0].decode()] = (offset, size, ptype)
    return table


def describe(path, what, offset, rel):
    data = path.read_bytes()
    return {"what": what, "path": rel, "offset": hex(offset), "size": len(data),
            "md5": hashlib.md5(data).hexdigest(), "sha256": hashlib.sha256(data).hexdigest()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", required=True, type=Path, help="R53_Mini_Kline_Canbus_Logger_Shiftlight checkout")
    ap.add_argument("--site", required=True, type=Path, help="gh-pages checkout of this repo")
    ap.add_argument("--notes", required=True)
    ap.add_argument("--build", type=int, help="replace this build instead of adding the next one")
    ap.add_argument("--date", default=datetime.date.today().isoformat())
    ap.add_argument("--no-build", action="store_true", help="use the images already built")
    ap.add_argument("--no-screens", action="store_true", help="the bridge only")
    args = ap.parse_args()
    repo, site = args.repo.expanduser().resolve(), args.site.expanduser().resolve()
    screens = [] if args.no_screens else SCREEN_ENVS

    dirty = git(repo, "status", "--porcelain", "--", *SOURCE_PATHS)
    if dirty:
        fail("uncommitted firmware changes:\n" + dirty)
    commit = git(repo, "rev-parse", "HEAD")
    main_cpp = (repo / BRIDGE_DIR / "src/main.cpp").read_text()
    firmware_build = int(re.search(r"#define\s+CAPS_FIRMWARE_BUILD\s+(\d+)", main_cpp).group(1))

    if not args.no_build:
        subprocess.run([pio(), "run", *sum((["-e", env] for env in BRIDGE_ENVS), [])], cwd=repo / BRIDGE_DIR, check=True)
        if screens:
            subprocess.run([pio(), "run", *sum((["-e", env] for env in screens), [])], cwd=repo / SCREEN_DIR, check=True)

    manifest_path = site / "releases.json"
    manifest = json.loads(manifest_path.read_text())
    releases = [r for r in manifest["releases"] if r["build"] != args.build]
    build = args.build or max(r["build"] for r in manifest["releases"]) + 1
    out = site / "firmware" / str(build)
    if out.exists():
        shutil.rmtree(out)
    boot_app0 = Path.home() / ".platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"

    boards = {}
    for env in BRIDGE_ENVS + screens:
        is_screen = env in SCREEN_ENVS
        built = repo / (SCREEN_DIR if is_screen else BRIDGE_DIR) / ".pio/build" / env
        dest = out / env
        dest.mkdir(parents=True)
        rel = lambda name: f"firmware/{build}/{env}/{name}"
        app_name = f"{env if is_screen else 'bridge-' + env}-build{build}.bin"  # screen-lcd21-build12.bin
        for src, name in [("bootloader.bin", "bootloader.bin"), ("partitions.bin", "partitions.bin"), ("firmware.bin", app_name)]:
            shutil.copyfile(built / src, dest / name)
        check_image(dest / "bootloader.bin", app=False)
        version = check_image(dest / app_name, app=True)
        table = read_partitions(dest / "partitions.bin")
        if table.get("app0", table.get("factory", (0,)))[0] != 0x10000:
            fail(f"{env}: no app at 0x10000 in its partition table")
        app = describe(dest / app_name, "app", 0x10000, rel(app_name))
        entry = {
            "sourceRepo": None,
            "sourceCommit": commit,
            "partitionTable": {"offset": "0x8000", "size": 3072,
                               "md5": hashlib.md5((dest / "partitions.bin").read_bytes()).hexdigest()},
            "partitions": {name: hex(offset) for name, (offset, _, _) in table.items()},
        }
        bootloader = describe(dest / "bootloader.bin", "bootloader", 0x0, rel("bootloader.bin"))
        partitions = describe(dest / "partitions.bin", "partitions", 0x8000, rel("partitions.bin"))
        if "otadata" in table:
            # The bridge: app0/app1, so the bootloader is pointed at app0 (publish-release.ps1 has why).
            if table["otadata"][0] != 0xE000:
                fail(f"{env}: otadata isn't at 0xe000")
            shutil.copyfile(boot_app0, dest / "boot_app0.bin")
            otadata = describe(dest / "boot_app0.bin", "otadata", 0xE000, rel("boot_app0.bin"))
            entry["firmwareBuild"] = firmware_build
            entry["update"] = [app, otadata]
            entry["fresh"] = [bootloader, partitions, otadata, app]
        else:
            # A screen: one factory app, and NVS runs over 0xe000, so no otadata ever.
            entry["screenVersion"] = version
            entry["update"] = [app]
            entry["fresh"] = [bootloader, partitions, app]
            if "bootclip" in table:
                # What `pio run -t upload` writes as well (bootclip.py), on Update too, so a new
                # clip reaches a screen that keeps its settings.
                clip_offset, clip_size, _ = table["bootclip"]
                shutil.copyfile(built / "bootclip.bin", dest / "bootclip.bin")
                clip = (dest / "bootclip.bin").read_bytes()
                if clip != CLIP_EMPTY:
                    if clip[:8] != CLIP_MAGIC or struct.unpack_from("<I", clip, 8)[0] != len(clip) - 16:
                        fail(f"{env}: bootclip.bin has no R5CLIP01 header or its length is wrong")
                    if clip[16:18] != b"\xff\xd8":
                        fail(f"{env}: the boot clip doesn't start with a JPEG frame")
                if len(clip) > clip_size:
                    fail(f"{env}: the boot clip is {len(clip)} bytes, its partition {clip_size}")
                bootclip = describe(dest / "bootclip.bin", "bootclip", clip_offset, rel("bootclip.bin"))
                entry["bootClip"] = clip != CLIP_EMPTY
                entry["update"].append(bootclip)
                entry["fresh"].append(bootclip)
        entry["ota"] = None
        boards[env] = entry
        clip_note = f", boot clip {entry['update'][-1]['size']} bytes" if entry.get("bootClip") else ""
        print(f"{env}: {app['size']} bytes, {'screen ' + version if is_screen else f'build {firmware_build}'}{clip_note}")

    releases.append({"build": build, "date": args.date, "bench": False, "notes": args.notes,
                     "sourceCommit": None, "boards": boards})
    manifest["releases"] = sorted(releases, key=lambda r: -r["build"])
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    for name in PAGE_FILES:
        shutil.copyfile(HERE / name, site / name)
    print(f"Build {build} from {commit[:7]} in {site}. Commit and push the site.")


if __name__ == "__main__":
    main()
