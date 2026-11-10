#!/usr/bin/env python3
"""Lint tracked compiler outputs and CI artifact upload retention."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

import yaml


_RESERVED_SUFFIXES = {
    ".o", ".obj", ".a", ".lib", ".pch", ".gch", ".profraw", ".profdata",
    ".gcda", ".gcno", ".ilk", ".idb", ".pdb", ".dwo", ".dSYM",
}
_MACHO_ELF_ARCHIVE_MAGIC = (
    b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe", b"\xfe\xed\xfa\xcf",
    b"\xfe\xed\xfa\xce", b"\x7fELF", b"!<arch>\n",
)
_COFF_MACHINES = {b"\x4c\x01", b"\x64\x86", b"\xaa\x64", b"\xc0\x01"}
_MAX_OBJ_FIXTURE_BYTES = 8 * 1024 * 1024
_WAVEFRONT_RECORDS = {"#", "####", "v", "vt", "vn", "f", "l", "p", "o", "g", "s", "mtllib", "usemtl"}
_GENERATED_TREES = {
    "build", "out", "_build", "cmake-build-debug", "cmake-build-release",
    "coverage", "coverage-out", "validation-artifacts", "validation-evidence",
    "validation-output", "evidence",
}
# Checked-in dependency link inputs; keep this allowlist exact and auditable.
_VENDORED_LINK_INPUTS = {
    "deps/GMP/gmp/lib/win-arm64/libgmp-10.lib",
    "deps/GMP/gmp/lib/win-x64/libgmp-10.lib",
    "deps/GMP/gmp/lib/win-x86/libgmp-10.lib",
    "deps/MPFR/mpfr/lib/win-arm64/libmpfr-4.lib",
    "deps/MPFR/mpfr/lib/win-x64/libmpfr-4.lib",
    "deps/MPFR/mpfr/lib/win-x86/libmpfr-4.lib",
    "deps/WebView2/lib/win-arm64/WebView2Loader.dll.lib",
    "deps/WebView2/lib/win-arm64/WebView2LoaderStatic.lib",
    "deps/WebView2/lib/win-x64/WebView2Loader.dll.lib",
    "deps/WebView2/lib/win-x64/WebView2LoaderStatic.lib",
    "deps/WebView2/lib/win-x86/WebView2Loader.dll.lib",
    "deps/WebView2/lib/win-x86/WebView2LoaderStatic.lib",
}


def _is_obj_binary(prefix: bytes) -> bool:
    return prefix.startswith(_MACHO_ELF_ARCHIVE_MAGIC) or prefix[:2] in _COFF_MACHINES


def _is_wavefront_fixture(path: Path) -> bool:
    """Accept bounded UTF-8 Wavefront text, never opaque or binary `.obj` files."""
    if path.stat().st_size > _MAX_OBJ_FIXTURE_BYTES:
        return False
    data = path.read_bytes()
    if not data or b"\0" in data:
        return False
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        return False
    records = [line.strip().split(None, 1)[0] for line in text.splitlines() if line.strip()]
    return bool(records) and all(record in _WAVEFRONT_RECORDS for record in records)


def _is_generated_tree(name: str) -> bool:
    parts = Path(name).parts
    for part in parts:
        lower = part.lower()
        if lower in _GENERATED_TREES or lower.endswith(".dsym"):
            return True
        if lower.startswith("cmake-build-") or re.fullmatch(r"(?:validation[-_](?:artifacts?|evidence|output|reports?))[-_]?v?[0-9][a-z0-9._-]*", lower):
            return True
    return False


def lint_tracked_build_outputs(repo_root: Path, git_files: list[str] | None = None) -> list[str]:
    """Reject tracked output suffixes, while allowing text Wavefront OBJ fixtures."""
    if git_files is None:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "ls-files", "-z"], check=True,
            stdout=subprocess.PIPE,
        )
        git_files = [item.decode("utf-8", "surrogateescape")
                     for item in result.stdout.split(b"\0") if item]
    findings = []
    for name in git_files:
        if _is_generated_tree(name):
            findings.append(f"{name}: tracked file inside a generated build/evidence tree")
            continue
        suffix = Path(name).suffix.lower()
        if suffix not in {item.lower() for item in _RESERVED_SUFFIXES}:
            continue
        path = repo_root / name
        try:
            if path.is_symlink():
                raise OSError("tracked output candidate is a symlink")
            resolved = path.resolve(strict=True)
            resolved.relative_to(repo_root.resolve())
            if not resolved.is_file():
                raise OSError("not a regular file")
            with resolved.open("rb") as stream:
                prefix = stream.read(16)
        except (OSError, ValueError) as error:
            findings.append(f"{name}: cannot safely inspect tracked output candidate: {error}")
            continue
        if name in _VENDORED_LINK_INPUTS:
            continue
        if suffix == ".obj":
            if _is_obj_binary(prefix):
                findings.append(f"{name}: tracked compiler/build output")
            elif _is_wavefront_fixture(resolved):
                continue
            else:
                findings.append(f"{name}: .obj file is not recognized text Wavefront geometry")
            continue
        findings.append(f"{name}: tracked compiler/build output")
    return findings


def _is_upload_action(value: str) -> bool:
    return re.search(r"(?:^|/)upload-artifact@[^\s]+$", value, re.IGNORECASE) is not None


def lint_upload_retention(workflow_paths: list[Path], repo_root: Path | None = None) -> list[str]:
    """Read real YAML so flow maps, anchors and scalar text retain their meaning."""
    findings = []
    for workflow in workflow_paths:
        try:
            if workflow.is_symlink():
                raise ValueError("workflow is a symlink")
            if repo_root is not None:
                workflow.resolve(strict=True).relative_to(repo_root.resolve())
            with workflow.open("rb") as stream:
                raw = stream.read(2 * 1024**2 + 1)
            if len(raw) > 2 * 1024**2:
                raise ValueError("workflow exceeds 2 MiB parsing budget")
            document = yaml.safe_load(raw.decode("utf-8"))
        except (OSError, UnicodeError, ValueError, yaml.YAMLError, RecursionError) as error:
            findings.append(f"{workflow}: cannot safely parse workflow: {error}")
            continue
        if not isinstance(document, dict) or not isinstance(document.get("jobs"), dict):
            findings.append(f"{workflow}: expected a workflow jobs mapping")
            continue
        for job_id, job in document["jobs"].items():
            if not isinstance(job, dict):
                findings.append(f"{workflow}: job '{job_id}' must be a mapping")
                continue
            steps = job.get("steps", [])
            if not isinstance(steps, list) or len(steps) > 10000:
                findings.append(f"{workflow}: job '{job_id}' has invalid or excessive steps")
                continue
            for number, step in enumerate(steps, 1):
                if not isinstance(step, dict):
                    findings.append(f"{workflow}: job '{job_id}' step {number} must be a mapping")
                    continue
                uses = step.get("uses", "")
                if not isinstance(uses, str) or not _is_upload_action(uses):
                    continue
                settings = step.get("with", {})
                value = settings.get("retention-days") if isinstance(settings, dict) else None
                positive_integer = (type(value) is int and value > 0) or (
                    isinstance(value, str) and re.fullmatch(r"[0-9]+", value) and int(value) > 0)
                if not positive_integer:
                    label = step.get("name", uses)
                    findings.append(
                        f"{workflow}: job '{job_id}' upload step '{label}' "
                        "requires a finite positive literal retention-days")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo", nargs="?", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    repo = args.repo.resolve()
    workflows = []
    for directory in (repo / ".github" / "workflows", repo / ".forgejo" / "workflows"):
        workflows.extend(sorted(directory.glob("*.yml")))
        workflows.extend(sorted(directory.glob("*.yaml")))
    findings = lint_tracked_build_outputs(repo) + lint_upload_retention(workflows, repo)
    for finding in findings:
        print(finding)
    if not findings:
        print("Validation artifact lint passed")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
