#!/usr/bin/env python3
"""Drop the builds a newer one has replaced from the flasher site.

Matt, 2026-11-26: "remove all the old builds from the web flasher, the ones that have been
replaced by newer versions". Each board keeps only its newest build. A release keeps the boards
it is the newest for and loses the rest; one with none left goes, with its images under
firmware/<build>/<env>/. Boards the page no longer offers (not in flasher.js's BOARDS, such as
the retired S3-Zero prototype) go too, because nothing can pick them. Git keeps every old image
in the site's history.

    python3 web-flasher/prune_releases.py --site ../esp32-shift-light-R53-mini-pages

Then commit and push the site. publish-r53.py runs this after each build it adds; after
publish-release.ps1, run it by hand.
"""
import argparse
import json
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def page_boards(page_js):
    """The board keys flasher.js offers: the entries of its BOARDS table."""
    text = page_js.read_text()
    table = text[text.index("const BOARDS"):text.index("function screenBoard")]
    keys = re.findall(r'^  "([a-z0-9_-]+)":', table, re.M)
    if not keys:
        sys.exit("prune_releases: no boards found in flasher.js")
    return set(keys)


def prune(site, page_js=HERE / "flasher.js"):
    """Prune site/releases.json and firmware/ in place. Returns what went, one line each."""
    manifest_path = site / "releases.json"
    manifest = json.loads(manifest_path.read_text())
    offered = page_boards(page_js)
    newest = {}
    for release in sorted(manifest["releases"], key=lambda r: -r["build"]):
        for board in release["boards"]:
            newest.setdefault(board, release["build"])
    gone, kept = [], []
    for release in sorted(manifest["releases"], key=lambda r: -r["build"]):
        build = release["build"]
        for board in list(release["boards"]):
            if board in offered and newest[board] == build:
                continue
            why = "not offered by the page" if board not in offered else f"replaced by build {newest[board]}"
            del release["boards"][board]
            shutil.rmtree(site / "firmware" / str(build) / board, ignore_errors=True)
            gone.append(f"build {build} {board}: {why}")
        if release["boards"]:
            kept.append(release)
        else:
            folder = site / "firmware" / str(build)
            if folder.exists() and not any(folder.iterdir()):
                folder.rmdir()
            gone.append(f"build {build}: nothing left in it")
    manifest["releases"] = kept
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return gone


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", required=True, type=Path, help="gh-pages checkout of this repo")
    args = ap.parse_args()
    site = args.site.expanduser().resolve()
    for line in prune(site):
        print(line)
    builds = [r["build"] for r in json.loads((site / "releases.json").read_text())["releases"]]
    print(f"Builds left: {', '.join(map(str, builds))}. Commit and push the site.")


if __name__ == "__main__":
    main()
