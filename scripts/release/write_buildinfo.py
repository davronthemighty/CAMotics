#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

"""Write release artifact metadata in a stable JSON format."""

import argparse
import hashlib
import json
import os
import platform
import subprocess
from pathlib import Path


def command(*args):
    try:
        return subprocess.check_output(args, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("artifacts", nargs="+", type=Path)
    args = parser.parse_args()

    files = []
    for path in sorted(args.artifacts, key=lambda item: item.name):
        files.append(
            {"name": path.name, "bytes": path.stat().st_size, "sha256": digest(path)}
        )

    info = {
        "schema": 1,
        "project": "CAMotics Fast",
        "release": args.tag,
        "revision": os.environ.get("GITHUB_SHA") or command("git", "rev-parse", "HEAD"),
        "source_date_epoch": os.environ.get("SOURCE_DATE_EPOCH"),
        "builder": {
            "os": platform.platform(),
            "compiler": command("g++", "--version"),
            "workflow_run": os.environ.get("GITHUB_RUN_ID"),
        },
        "artifacts": files,
    }
    args.output.write_text(
        json.dumps(info, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
