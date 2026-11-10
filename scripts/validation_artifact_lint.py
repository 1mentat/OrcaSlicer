#!/usr/bin/env python3
"""Lint tracked compiler outputs and CI artifact upload retention."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path


_RESERVED_SUFFIXES = {
    ".o", ".obj", ".a", ".lib", ".pch", ".gch", ".profraw", ".profdata",
    ".gcda", ".gcno", ".ilk", ".idb", ".pdb", ".dwo", ".dSYM",
}
_MACHO_ELF_ARCHIVE_MAGIC = (
    b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe", b"\xfe\xed\xfa\xcf",
    b"\xfe\xed\xfa\xce", b"\x7fELF", b"!<arch>\n",
)
_COFF_MACHINES = {b"\x4c\x01", b"\x64\x86", b"\xaa\x64", b"\xc0\x01"}
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
        suffix = Path(name).suffix.lower()
        if suffix not in {item.lower() for item in _RESERVED_SUFFIXES}:
            continue
        if name in _VENDORED_LINK_INPUTS:
            continue
        path = repo_root / name
        try:
            resolved = path.resolve(strict=True)
            resolved.relative_to(repo_root.resolve())
            if not resolved.is_file():
                raise OSError("not a regular file")
            with resolved.open("rb") as stream:
                prefix = stream.read(16)
        except (OSError, ValueError) as error:
            findings.append(f"{name}: cannot safely inspect tracked output candidate: {error}")
            continue
        if suffix == ".obj" and not _is_obj_binary(prefix):
            continue
        findings.append(f"{name}: tracked compiler/build output")
    return findings


def _step_blocks(lines: list[str]):
    """Yield indentation-scoped step entries from every GitHub-style steps sequence."""
    for index, line in enumerate(lines):
        match = re.match(r"^(\s*)steps:\s*(?:#.*)?$", line)
        if not match:
            continue
        steps_indent = len(match.group(1))
        item_indent = steps_indent + 2
        start = index + 1
        while start < len(lines):
            if lines[start].strip() and len(lines[start]) - len(lines[start].lstrip()) <= steps_indent:
                break
            if not lines[start].strip() or lines[start].lstrip().startswith("#"):
                start += 1
                continue
            indent = len(lines[start]) - len(lines[start].lstrip())
            if indent != item_indent or not re.match(r"-\s+", lines[start].lstrip()):
                # The first non-list entry means this is not a supported step sequence.
                yield start + 1, [lines[start]], "unsupported steps sequence"
                break
            end = start + 1
            while end < len(lines):
                candidate = lines[end]
                if candidate.strip() and len(candidate) - len(candidate.lstrip()) <= steps_indent:
                    break
                if candidate.strip() and len(candidate) - len(candidate.lstrip()) == item_indent and re.match(r"-\s+", candidate.lstrip()):
                    break
                end += 1
            yield start + 1, lines[start:end], None
            start = end


def _step_key(block: list[str], key: str) -> tuple[int, str] | None:
    for index, line in enumerate(block):
        match = re.match(rf"^\s*(?:-\s+)?{re.escape(key)}:\s*(.*?)\s*(?:#.*)?$", line)
        if match:
            return index, match.group(1).strip(" '\"")
    return None


def _retention_value(block: list[str]) -> tuple[bool, str | None]:
    with_entry = _step_key(block, "with")
    if with_entry is None:
        return False, None
    index, inline = with_entry
    if inline:
        # Inline flow maps are valid YAML but cannot be safely scoped by this parser.
        return False, None
    with_indent = len(block[index]) - len(block[index].lstrip())
    for line in block[index + 1:]:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if indent <= with_indent:
            break
        match = re.match(r"^\s+retention-days:\s*(.*?)\s*(?:#.*)?$", line)
        if match:
            return True, match.group(1).strip(" '\"")
    return True, None


def _is_upload_action(value: str) -> bool:
    value = value.strip(" '\"")
    # Accept normal action references and absolute Forgejo action URLs, but not
    # download-artifact or similarly named actions.
    return re.search(r"(?:^|/)upload-artifact@[^\s]+$", value, re.IGNORECASE) is not None


def lint_upload_retention(workflow_paths: list[Path]) -> list[str]:
    findings = []
    for workflow in workflow_paths:
        try:
            lines = workflow.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as error:
            findings.append(f"{workflow}: cannot read workflow: {error}")
            continue
        for start_line, block, error in _step_blocks(lines):
            if error:
                findings.append(f"{workflow}:{start_line}: {error}")
                continue
            uses = _step_key(block, "uses")
            if uses is None or not _is_upload_action(uses[1]):
                continue
            name = _step_key(block, "name")
            label = name[1] if name and name[1] else uses[1]
            has_with, value = _retention_value(block)
            if not has_with or value is None or not re.fullmatch(r"[0-9]+", value) or int(value) <= 0:
                findings.append(f"{workflow}:{start_line}: upload step '{label}' requires a finite positive literal retention-days")
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
    findings = lint_tracked_build_outputs(repo) + lint_upload_retention(workflows)
    for finding in findings:
        print(finding)
    if not findings:
        print("Validation artifact lint passed")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
