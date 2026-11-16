"""Check bounded CI upload inputs without changing upload-artifact's paths/layout.

This is a stat preflight, not a reservation or lock: the upload action reads the
original paths afterwards. Hosted runners supply disposal; self-hosted jobs must
provide the existing authenticated validation_artifacts lease.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import fnmatch
from functools import lru_cache
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

MAX_BYTES = 4 * 1024**3


class UploadError(ValueError):
    pass


def _inside(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _integer(value: str, name: str, low: int, high: int) -> int:
    if not re.fullmatch(r"[0-9]+", value) or not low <= int(value) <= high:
        raise UploadError(f"{name} must be an integer from {low} to {high}")
    return int(value)


def _boolean(value: str, name: str) -> bool:
    if value not in {"true", "false"}:
        raise UploadError(f"{name} must be true or false")
    return value == "true"


def _text(value: str, name: str) -> str:
    if not value or any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise UploadError(f"{name} must be nonempty text without control characters")
    return value


def _head(workspace: Path) -> str:
    # Git context variables must not redirect identity away from this checkout.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    result = subprocess.run(["git", "-C", str(workspace), "rev-parse", "--verify", "HEAD^{commit}"],
                            env=env, capture_output=True, text=True, timeout=10, check=True)
    value = result.stdout.strip()
    if not re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", value):
        raise UploadError("checkout HEAD is not a full commit identity")
    return value


@lru_cache(maxsize=8192)
def _matches(parts: tuple[str, ...], pattern: tuple[str, ...], prefix: bool = False) -> bool:
    # An exactly matched directory includes its descendants. Prefix matching
    # prunes directories that cannot contain a selected file, before traversal.
    if not pattern:
        return True
    if not parts:
        return prefix or all(piece == "**" for piece in pattern)
    if pattern[0] == "**":
        return (_matches(parts, pattern[1:], prefix) or
                _matches(parts[1:], pattern, prefix))
    return fnmatch.fnmatchcase(os.path.normcase(parts[0]), os.path.normcase(pattern[0])) and \
        _matches(parts[1:], pattern[1:], prefix)


def _pattern(value: str, workspace: Path, roots: tuple[Path, ...]) -> tuple[Path, tuple[str, ...]]:
    if "\x00" in value:
        raise UploadError("path contains NUL")
    path = Path(value).expanduser()
    if ".." in path.parts:
        raise UploadError("parent path traversal is not allowed")
    if not path.is_absolute():
        path = workspace / path
    parts = path.parts
    split = next((i for i, piece in enumerate(parts) if any(c in piece for c in "*?[")), len(parts))
    base = Path(*parts[:split])
    canonical = base.resolve()
    if not any(_inside(canonical, root) for root in roots):
        raise UploadError("upload path escapes allowed CI storage")
    return base, parts[split:]


def _files(patterns: str, workspace: Path, roots: tuple[Path, ...], hidden: bool,
           budget: int) -> tuple[int, int]:
    positive = []
    negative = []
    for line in patterns.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        excluded = line.startswith("!")
        value = line[1:] if excluded else line
        if not value:
            raise UploadError("empty exclusion pattern")
        base, tail = _pattern(value, workspace, roots)
        (negative if excluded else positive).append((base, tail))
    selected: set[Path] = set()
    total = 0

    def visit(path: Path, base: Path, pattern: tuple[str, ...], ancestors: frozenset[Path]) -> None:
        nonlocal total
        relative = path.relative_to(base).parts
        if not _matches(relative, pattern, True):
            return
        for excluded_base, excluded_pattern in negative:
            if _inside(path, excluded_base) and _matches(path.relative_to(excluded_base).parts, excluded_pattern):
                return
        if not hidden and any(piece.startswith(".") for root in roots if _inside(path, root)
                              for piece in path.relative_to(root).parts):
            return
        try:
            info = path.lstat()
        except FileNotFoundError:
            return
        canonical = path.resolve(strict=True)
        if not any(_inside(canonical, root) for root in roots):
            raise UploadError("selected symlink escapes allowed CI storage")
        if stat.S_ISLNK(info.st_mode):
            info = path.stat()
        if stat.S_ISDIR(info.st_mode):
            if canonical in ancestors:
                raise UploadError("upload directory symlink cycle")
            with os.scandir(path) as entries:
                for entry in entries:
                    visit(path / entry.name, base, pattern, ancestors | {canonical})
        elif stat.S_ISREG(info.st_mode):
            if _matches(relative, pattern) and path not in selected:
                selected.add(path)
                total += info.st_size
                if total > budget:
                    raise UploadError("actual selected upload bytes exceed max-bytes")
        else:
            raise UploadError("selected upload contains a special file")

    for base, pattern in positive:
        visit(base, base, pattern, frozenset())
    return len(selected), total


def preflight(env: dict[str, str]) -> dict:
    workspace = Path(env["GITHUB_WORKSPACE"]).resolve(strict=True)
    temporary = Path(env["RUNNER_TEMP"]).resolve(strict=True)
    if not workspace.is_dir() or not temporary.is_dir():
        raise UploadError("workspace and runner temp must be directories")
    head = _head(workspace)
    actor = _text(env.get("GITHUB_ACTOR", ""), "CI owner")
    repository = _text(env.get("GITHUB_REPOSITORY", ""), "CI repository")
    run = _text(env.get("GITHUB_RUN_ID", ""), "CI run")
    job = _text(env.get("GITHUB_JOB", ""), "CI job")
    attempt = _text(env.get("GITHUB_RUN_ATTEMPT", "1"), "CI attempt")
    name = _text(env.get("VALIDATION_UPLOAD_NAME", ""), "artifact name")
    days = _integer(env.get("VALIDATION_UPLOAD_RETENTION_DAYS", "14"), "retention-days", 1, 30)
    budget = _integer(env.get("VALIDATION_UPLOAD_MAX_BYTES", str(MAX_BYTES)), "max-bytes", 1, MAX_BYTES)
    _integer(env.get("VALIDATION_UPLOAD_COMPRESSION_LEVEL", "6"), "compression-level", 0, 9)
    hidden = _boolean(env.get("VALIDATION_UPLOAD_INCLUDE_HIDDEN_FILES", "false"), "include-hidden-files")
    _boolean(env.get("VALIDATION_UPLOAD_OVERWRITE", "false"), "overwrite")
    policy = env.get("VALIDATION_UPLOAD_IF_NO_FILES_FOUND", "warn")
    if policy not in {"warn", "error", "ignore"}:
        raise UploadError("if-no-files-found must be warn, error or ignore")
    roots = (workspace, temporary)
    disposal = "github-hosted provider disposal"
    if env.get("RUNNER_ENVIRONMENT") != "github-hosted":
        import validation_artifacts as artifacts
        required = ("VALIDATION_ARTIFACT_ROOT", "VALIDATION_ARTIFACT_RUN_ID", "VALIDATION_ARTIFACT_TOKEN",
                    "VALIDATION_ARTIFACT_DIR", "VALIDATION_SOURCE_SHA")
        if any(not env.get(key) for key in required):
            raise UploadError("non-hosted uploads require an authenticated artifact manager context")
        receipt = artifacts.check(env[required[0]], env[required[1]], env[required[2]])
        payload = Path(receipt["payload_dir"]).resolve(strict=True)
        if payload != Path(env["VALIDATION_ARTIFACT_DIR"]).resolve(strict=True) or \
                head != env["VALIDATION_SOURCE_SHA"] or head != receipt["source_sha"]:
            raise UploadError("manager payload/source ownership mismatch")
        roots = (payload,)
        disposal = "authenticated artifact manager lease"
    count, size = _files(env.get("VALIDATION_UPLOAD_PATH", ""), workspace, roots, hidden, budget)
    if not count and policy == "error":
        raise UploadError("no upload files found")
    now = datetime.now(timezone.utc)
    return {"artifact": name, "work_item": f"{repository}/runs/{run}/attempts/{attempt}/jobs/{job}",
            "source_sha": head, "owner": actor, "checked_at": now.isoformat(),
            "expires_at": (now + timedelta(days=days)).isoformat(), "retention_days": days,
            "max_bytes": budget, "selected_bytes": size, "selected_files": count,
            "empty_policy": policy, "local_disposal": disposal,
            "budget_enforcement": "actual stat preflight; files are not locked until upload"}


def main() -> int:
    try:
        receipt = preflight(dict(os.environ))
        text = json.dumps(receipt, sort_keys=True)
        if not receipt["selected_files"] and receipt["empty_policy"] == "warn":
            print("::warning::No upload files found by validation preflight", file=sys.stderr)
        summary = os.environ.get("GITHUB_STEP_SUMMARY")
        if summary:
            # Runner-created summary only; no extra manifest or artifact changes.
            path = Path(summary)
            roots = [Path(os.environ[k]).resolve(strict=True) for k in ("GITHUB_WORKSPACE", "RUNNER_TEMP")]
            if not any(_inside(path.resolve(), root) for root in roots):
                raise UploadError("step summary escapes CI storage")
            fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0), 0o600)
            try:
                if not stat.S_ISREG(os.fstat(fd).st_mode):
                    raise UploadError("step summary is not a regular file")
                safe = text.replace("`", "\\u0060").replace("<", "\\u003c")
                os.write(fd, ("### Validation upload preflight\n\n```json\n" + safe + "\n```\n").encode())
            finally:
                os.close(fd)
        print(text)
        return 0
    except (UploadError, KeyError, OSError, subprocess.SubprocessError, ValueError, RuntimeError) as error:
        print(f"validation upload: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
