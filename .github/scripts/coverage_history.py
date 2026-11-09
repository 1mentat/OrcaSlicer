#!/usr/bin/env python3
"""Keep small coverage metadata on a data branch, separate from build artifacts."""

import argparse
import base64
from datetime import datetime
import json
import os
from pathlib import Path
import re
from urllib.error import HTTPError
from urllib.request import Request, urlopen


BRANCH = "coverage-data"
MAX_BYTES = 16 * 1024 * 1024


class GitHub:
    def __init__(self):
        repository = os.environ["GITHUB_REPOSITORY"]
        if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository):
            raise ValueError("invalid repository identity")
        self.base = "https://api.github.com/repos/" + repository
        self.token = os.environ["GH_TOKEN"]

    def request(self, path, body=None, method="GET", missing_ok=False):
        data = json.dumps(body).encode() if body is not None else None
        request = Request(self.base + path, data=data, method=method, headers={
            "Authorization": "Bearer " + self.token,
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2026-03-10",
            "Content-Type": "application/json",
        })
        try:
            with urlopen(request, timeout=60) as response:
                content = response.read(MAX_BYTES + 1)
        except HTTPError as error:
            if missing_ok and error.code == 404:
                return None
            # Do not echo request headers, credentials, or server response bodies.
            raise RuntimeError(f"GitHub metadata request failed: HTTP {error.code}") from None
        if len(content) > MAX_BYTES:
            raise ValueError("coverage metadata exceeds its size limit")
        return json.loads(content)

    def head(self):
        value = self.request("/git/ref/heads/" + BRANCH, missing_ok=True)
        return value["object"]["sha"] if value else None

    def file(self, name, revision):
        value = self.request(f"/contents/{name}?ref={revision}")
        if value.get("encoding") == "none":
            value = self.request("/git/blobs/" + value["sha"])
        return json.loads(base64.b64decode(value["content"], validate=False))


def validate(summary, history, expected_commit):
    if summary.get("schema_version") != 1 or summary.get("valid") is not True:
        raise ValueError("only valid versioned coverage measurements may be published")
    if not re.fullmatch(r"[0-9a-f]{40}", expected_commit) or summary.get("commit") != expected_commit:
        raise ValueError("report does not identify the measured source commit")
    if summary.get("source_verified") is not True or summary.get("integrity_status") != "pass":
        raise ValueError("source or mapping integrity was not verified")
    if not isinstance(history, list) or not 1 <= len(history) <= 365:
        raise ValueError("history must contain 1–365 valid observations")
    if history != summary.get("history"):
        raise ValueError("history does not match the report")
    configuration = summary.get("configuration_id", "")
    if not re.fullmatch(r"[0-9a-f]{64}", configuration):
        raise ValueError("invalid measurement configuration identity")
    for observation in history:
        if observation.get("valid") is not True or not re.fullmatch(r"[0-9a-f]{40}", observation.get("commit", "")):
            raise ValueError("invalid history observation")
        if observation.get("configuration_id") != configuration:
            raise ValueError("history contains incompatible measurement configurations")
        when = datetime.fromisoformat(observation["measured_at"].replace("Z", "+00:00"))
        if when.tzinfo is None:
            raise ValueError("history times must include a timezone")
        for metric in observation["summary"].values():
            if metric is not None and (type(metric["covered"]) is not int
                                       or type(metric["count"]) is not int
                                       or not 0 <= metric["covered"] <= metric["count"]):
                raise ValueError("invalid history counts")
    if not any(value["commit"] == expected_commit
               and value["measured_at"] == summary["measured_at"] for value in history):
        raise ValueError("the current measurement is absent from history")
    measured = datetime.fromisoformat(summary["measured_at"].replace("Z", "+00:00"))
    if measured.tzinfo is None:
        raise ValueError("measurement time must include a timezone")
    return measured


def publish(api, directory, expected_commit):
    summary = json.loads((directory / "summary.json").read_text())
    history = json.loads((directory / "history.json").read_text())
    measured = validate(summary, history, expected_commit)
    parent = api.head()
    if parent:
        current = api.file("summary.json", parent)
        previous_time = datetime.fromisoformat(current["measured_at"].replace("Z", "+00:00"))
        if previous_time >= measured:
            print("A newer or identical measurement is already published; keeping it.")
            return False
        previous_commit = current.get("commit", "")
        if not re.fullmatch(r"[0-9a-f]{40}", previous_commit):
            raise ValueError("published history has an invalid source commit")
        if previous_commit != expected_commit:
            comparison = api.request(f"/compare/{previous_commit}...{expected_commit}?per_page=1")
            if comparison.get("status") != "ahead":
                print("This source does not descend from the published baseline; keeping the newer source.")
                return False
    tree = []
    for name, value in (("history.json", history), ("summary.json", summary)):
        content = base64.b64encode((json.dumps(value, separators=(",", ":")) + "\n").encode()).decode()
        blob = api.request("/git/blobs", {"content": content, "encoding": "base64"}, "POST")
        tree.append({"path": name, "mode": "100644", "type": "blob", "sha": blob["sha"]})
    new_tree = api.request("/git/trees", {"tree": tree}, "POST")
    commit = api.request("/git/commits", {
        "message": "Record core coverage for " + expected_commit[:12],
        "tree": new_tree["sha"], "parents": [parent] if parent else [],
    }, "POST")
    if parent:
        api.request("/git/refs/heads/" + BRANCH, {"sha": commit["sha"], "force": False}, "PATCH")
    else:
        api.request("/git/refs", {"ref": "refs/heads/" + BRANCH, "sha": commit["sha"]}, "POST")
    print("Recorded coverage history for " + expected_commit[:12])
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("fetch", "publish"))
    parser.add_argument("directory", type=Path)
    parser.add_argument("--commit")
    args = parser.parse_args()
    api = GitHub()
    if args.operation == "publish":
        promoted = publish(api, args.directory, args.commit or "")
        if os.environ.get("GITHUB_OUTPUT"):
            with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
                output.write("promoted=" + str(promoted).lower() + "\n")
    else:
        head = api.head()
        args.directory.mkdir(parents=True, exist_ok=True)
        if head:
            for name in ("history.json", "summary.json"):
                (args.directory / name).write_text(json.dumps(api.file(name, head)))
        else:
            print("No coverage data branch exists yet; this run can establish the first baseline.")


if __name__ == "__main__":
    main()
