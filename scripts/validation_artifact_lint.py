#!/usr/bin/env python3
"""Lint tracked compiler outputs and CI artifact upload retention."""

from __future__ import annotations

import argparse
import ast
import json
import re
import subprocess
from pathlib import Path, PurePosixPath

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
_EXPECTED_CLI_PRODUCERS = {
    "settings-inventory": ("scripts/settings_inventory.py", "--out"),
    "mutation-validation": ("scripts/slicer_mutation.py", "--out"),
    "pr-validation": ("scripts/slicer_pr_validation.py", "--output-dir"),
    "module-feedback": ("scripts/slicer_module_feedback.py", "--out"),
}
_EXPECTED_UPLOAD_PRODUCER = {
    "id": "public-validation-upload",
    "path": ".github/actions/validation-upload/action.yml",
    "kind": "github-upload",
    "lifecycle": "ephemeral-runner",
    "retention_days": 14,
    "max_bytes": 4 * 1024**3,
}
_UPLOAD_INPUT_ENV = {
    "VALIDATION_UPLOAD_NAME": "${{ inputs.name }}",
    "VALIDATION_UPLOAD_PATH": "${{ inputs.path }}",
    "VALIDATION_UPLOAD_RETENTION_DAYS": "${{ inputs.retention-days }}",
    "VALIDATION_UPLOAD_MAX_BYTES": "${{ inputs.max-bytes }}",
    "VALIDATION_UPLOAD_IF_NO_FILES_FOUND": "${{ inputs.if-no-files-found }}",
    "VALIDATION_UPLOAD_OVERWRITE": "${{ inputs.overwrite }}",
    "VALIDATION_UPLOAD_COMPRESSION_LEVEL": "${{ inputs.compression-level }}",
    "VALIDATION_UPLOAD_INCLUDE_HIDDEN_FILES": "${{ inputs.include-hidden-files }}",
}
_UPLOAD_WITH_INPUTS = {
    "name": "${{ inputs.name }}",
    "path": "${{ inputs.path }}",
    "retention-days": "${{ inputs.retention-days }}",
    "if-no-files-found": "${{ inputs.if-no-files-found }}",
    "overwrite": "${{ inputs.overwrite }}",
    "compression-level": "${{ inputs.compression-level }}",
    "include-hidden-files": "${{ inputs.include-hidden-files }}",
}
_ABSENT = object()


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


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _safe_repo_file(repo_root: Path, relative: str) -> tuple[Path | None, str | None]:
    if not isinstance(relative, str) or "\\" in relative:
        return None, "path must be a repository-relative POSIX path"
    parsed = PurePosixPath(relative)
    if parsed.is_absolute() or not parsed.parts or any(part in ("", ".", "..") for part in relative.split("/")):
        return None, "path must be confined to the repository"
    root = repo_root.resolve()
    path = root.joinpath(*parsed.parts)
    try:
        for candidate in (path, *path.parents):
            if candidate == root.parent:
                break
            if candidate.is_symlink():
                return None, "path includes a symlink"
        resolved = path.resolve(strict=True)
        resolved.relative_to(root)
        if not resolved.is_file():
            return None, "path is not a regular file"
    except (OSError, ValueError):
        return None, "path is missing or escapes the repository"
    return resolved, None


def _main_guard(test: ast.expr) -> bool:
    return (isinstance(test, ast.Compare) and len(test.ops) == 1
            and isinstance(test.ops[0], ast.Eq) and len(test.comparators) == 1
            and ((isinstance(test.left, ast.Name) and test.left.id == "__name__"
                  and isinstance(test.comparators[0], ast.Constant) and test.comparators[0].value == "__main__")
                 or (isinstance(test.comparators[0], ast.Name) and test.comparators[0].id == "__name__"
                     and isinstance(test.left, ast.Constant) and test.left.value == "__main__")))


def _has_managed_entrypoint(path: Path, expected_option: str) -> tuple[bool, str]:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, UnicodeError, SyntaxError) as error:
        return False, f"cannot parse producer source ({type(error).__name__})"

    runner_aliases = set()
    module_aliases = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "validation_runner":
            for alias in node.names:
                if alias.name == "managed_main":
                    runner_aliases.add(alias.asname or alias.name)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "validation_runner":
                    module_aliases.add(alias.asname or alias.name.split(".")[0])

    guards = [node for node in tree.body if isinstance(node, ast.If) and _main_guard(node.test)]
    calls = []
    for guard in guards:
        for node in ast.walk(guard):
            if not isinstance(node, ast.Call):
                continue
            target = node.func
            is_managed = ((isinstance(target, ast.Name) and target.id in runner_aliases)
                          or (isinstance(target, ast.Attribute) and target.attr == "managed_main"
                              and isinstance(target.value, ast.Name) and target.value.id in module_aliases))
            if is_managed:
                calls.append((guard, node))

    if not runner_aliases and not module_aliases:
        return False, "producer must import managed_main from validation_runner"
    if len(calls) != 1:
        return False, "producer __main__ must execute exactly one managed_main call"
    guard, call = calls[0]
    if (not call.args or not isinstance(call.args[0], ast.Name) or call.args[0].id != "main"
            or not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "main"
                       for node in tree.body)):
        return False, "producer __main__ must pass its executable main function to managed_main"
    option = next((keyword.value for keyword in call.keywords if keyword.arg == "output_option"), None)
    if not isinstance(option, ast.Constant) or option.value != expected_option:
        return False, f"producer managed_main must bind output_option={expected_option!r}"
    wrapped = any(
        isinstance(statement, ast.Raise)
        and isinstance(statement.exc, ast.Call)
        and isinstance(statement.exc.func, ast.Name)
        and statement.exc.func.id == "SystemExit"
        and call in ast.walk(statement.exc)
        for statement in guard.body
    )
    if not wrapped:
        return False, "producer __main__ must exit with its managed_main result"
    return True, ""


def _literal_bound(value, default: int, maximum: int, validated_expression: str) -> tuple[bool, int | None]:
    if value is _ABSENT:
        return True, default
    if type(value) is int:
        number = value
    elif isinstance(value, str) and re.fullmatch(r"[0-9]+", value):
        number = int(value)
    elif value == validated_expression:
        return True, None
    else:
        return False, None
    return (1 <= number <= maximum), number


def _lint_upload_checker(path: Path, expected_max_bytes: int) -> list[str]:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, UnicodeError, SyntaxError) as error:
        return [f"{path}: cannot parse upload checker ({type(error).__name__})"]
    values = []
    for node in tree.body:
        targets = node.targets if isinstance(node, ast.Assign) else [node.target] if isinstance(node, ast.AnnAssign) else []
        if any(isinstance(target, ast.Name) and target.id == "MAX_BYTES" for target in targets):
            value = _constant_integer(node.value)
            if value is not None:
                values.append(value)
    if values != [expected_max_bytes]:
        return [f"{path}: MAX_BYTES must be the registry's exact {expected_max_bytes} byte limit"]
    if not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "preflight"
               for node in tree.body):
        return [f"{path}: missing executable preflight function"]
    return []


def _constant_integer(node: ast.expr) -> int | None:
    if isinstance(node, ast.Constant) and type(node.value) is int:
        return node.value
    if not isinstance(node, ast.BinOp):
        return None
    left = _constant_integer(node.left)
    right = _constant_integer(node.right)
    if left is None or right is None or abs(left) > 2**63 or abs(right) > 2**63:
        return None
    if isinstance(node.op, ast.Pow) and right > 16:
        return None
    operations = {
        ast.Add: lambda: left + right,
        ast.Sub: lambda: left - right,
        ast.Mult: lambda: left * right,
        ast.Pow: lambda: left ** right,
        ast.FloorDiv: lambda: left // right if right else 0,
    }
    operation = operations.get(type(node.op))
    if operation is None:
        return None
    try:
        result = operation()
    except (ArithmeticError, OverflowError):
        return None
    return result if abs(result) <= 2**63 else None


def _lint_upload_action(repo_root: Path, action_path: Path, retention_days: int, max_bytes: int) -> list[str]:
    findings = []
    try:
        raw = action_path.read_bytes()
        if len(raw) > 2 * 1024**2:
            raise ValueError("action exceeds 2 MiB parsing budget")
        action = yaml.safe_load(raw.decode("utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError, ValueError, RecursionError) as error:
        return [f"{action_path}: cannot safely parse validation upload action ({type(error).__name__})"]
    if not isinstance(action, dict):
        return [f"{action_path}: expected a composite action mapping"]

    inputs = action.get("inputs")
    if not isinstance(inputs, dict):
        findings.append(f"{action_path}: missing composite action inputs")
        inputs = {}
    for name in ("name", "path"):
        if not isinstance(inputs.get(name), dict) or inputs[name].get("required") is not True:
            findings.append(f"{action_path}: input '{name}' must be required")
    expected_defaults = {"retention-days": retention_days, "max-bytes": max_bytes}
    for name, expected in expected_defaults.items():
        setting = inputs.get(name)
        value = setting.get("default") if isinstance(setting, dict) else None
        if not (type(value) is int and value == expected) and not (isinstance(value, str) and value == str(expected)):
            findings.append(f"{action_path}: input '{name}' default must match registry value {expected}")

    runs = action.get("runs")
    steps = runs.get("steps") if isinstance(runs, dict) else None
    if not isinstance(runs, dict) or runs.get("using") != "composite" or not isinstance(steps, list):
        return findings + [f"{action_path}: expected composite runs with a steps list"]
    if any(not isinstance(step, dict) for step in steps):
        findings.append(f"{action_path}: each composite step must be a mapping")
    preflights = [
        (index, step) for index, step in enumerate(steps)
        if isinstance(step, dict) and isinstance(step.get("run"), str)
        and "scripts/validation_upload.py" in step["run"]
    ]
    upload_steps = [
        (index, step) for index, step in enumerate(steps)
        if isinstance(step, dict) and isinstance(step.get("uses"), str)
        and re.fullmatch(r"actions/upload-artifact@[^\s]+", step["uses"])
    ]
    if len(preflights) != 1:
        findings.append(f"{action_path}: requires exactly one validation_upload.py preflight step")
    if len(upload_steps) != 1:
        findings.append(f"{action_path}: requires exactly one SHA-pinned upload-artifact step")
    elif not re.fullmatch(r"actions/upload-artifact@[0-9a-fA-F]{40}", upload_steps[0][1]["uses"]):
        findings.append(f"{action_path}: upload-artifact must be pinned to a full commit SHA")
    if len(preflights) == 1:
        preflight_index, preflight = preflights[0]
        if len(upload_steps) == 1 and preflight_index >= upload_steps[0][0]:
            findings.append(f"{action_path}: byte/ownership preflight must run before upload-artifact")
        command = preflight["run"].strip()
        if not re.fullmatch(r"python(?:3)?\s+[\"']?\$GITHUB_ACTION_PATH/(?:\.\./){3}scripts/validation_upload\.py[\"']?", command):
            findings.append(f"{action_path}: preflight must invoke the tracked upload checker from the repository")
        else:
            checker, checker_error = _safe_repo_file(repo_root, "scripts/validation_upload.py")
            if checker_error:
                findings.append(f"{action_path}: upload checker {checker_error}")
            elif (action_path.parent / "../../../scripts/validation_upload.py").resolve() != checker:
                findings.append(f"{action_path}: preflight path does not resolve to tracked scripts/validation_upload.py")
        if "if" in preflight:
            findings.append(f"{action_path}: preflight cannot be conditionally skipped")
        if preflight.get("shell") != "bash":
            findings.append(f"{action_path}: preflight must use the declared bash shell")
        if preflight.get("continue-on-error") is not None and preflight.get("continue-on-error") is not False:
            findings.append(f"{action_path}: preflight cannot continue after an error")
        env = preflight.get("env")
        if not isinstance(env, dict):
            findings.append(f"{action_path}: preflight must wire all validation upload inputs through env")
        else:
            for key, value in _UPLOAD_INPUT_ENV.items():
                if env.get(key) != value:
                    findings.append(f"{action_path}: preflight env '{key}' must bind {value}")

    if len(upload_steps) == 1 and re.fullmatch(r"actions/upload-artifact@[0-9a-fA-F]{40}", upload_steps[0][1]["uses"]):
        upload = upload_steps[0][1]
        condition = upload.get("if")
        if condition is not None and condition not in ("success()", "${{ success() }}"):
            findings.append(f"{action_path}: upload step cannot run after a failed preflight")
        settings = upload.get("with")
        if not isinstance(settings, dict):
            findings.append(f"{action_path}: upload step must forward the wrapper inputs")
        else:
            for key, value in _UPLOAD_WITH_INPUTS.items():
                if settings.get(key) != value:
                    findings.append(f"{action_path}: upload '{key}' must forward {value}")
    return findings


def _lint_wrapper_callers(repo_root: Path, workflow_paths: list[Path], retention_default: int,
                          max_bytes: int) -> list[str]:
    findings = []
    for workflow in workflow_paths:
        try:
            raw = workflow.read_bytes()
            if len(raw) > 2 * 1024**2:
                raise ValueError("workflow exceeds 2 MiB parsing budget")
            document = yaml.safe_load(raw.decode("utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError, ValueError, RecursionError):
            continue  # The ordinary upload linter reports the parse error.
        jobs = document.get("jobs") if isinstance(document, dict) else None
        if not isinstance(jobs, dict):
            continue
        for job_id, job in jobs.items():
            steps = job.get("steps") if isinstance(job, dict) else None
            if not isinstance(steps, list):
                continue
            for number, step in enumerate(steps, 1):
                if not isinstance(step, dict) or step.get("uses") != "./.github/actions/validation-upload":
                    continue
                location = f"{workflow}: job '{job_id}' wrapper step {number}"
                if step.get("continue-on-error") not in (None, False):
                    findings.append(f"{location}: cannot ignore validation preflight failures")
                settings = step.get("with")
                if not isinstance(settings, dict):
                    findings.append(f"{location}: requires a with mapping")
                    continue
                for name in ("name", "path"):
                    if not isinstance(settings.get(name), str) or not settings[name].strip():
                        findings.append(f"{location}: requires nonempty '{name}' input")
                retention_ok, _ = _literal_bound(settings.get("retention-days", _ABSENT), retention_default, 30, "${{ inputs.retention-days }}")
                if not retention_ok:
                    findings.append(f"{location}: retention-days must be 1..30 or the validated inputs.retention-days value")
                budget_ok, _ = _literal_bound(settings.get("max-bytes", _ABSENT), max_bytes, max_bytes, "${{ inputs.max-bytes }}")
                if not budget_ok:
                    findings.append(f"{location}: max-bytes must be 1..{max_bytes} or the validated inputs.max-bytes value")
    return findings


def lint_producer_registry(repo_root: Path, workflow_paths: list[Path] | None = None) -> list[str]:
    """Validate registry schema, executable producer wiring, and upload wrapper contract.

    Upload byte checks are a stat preflight before upload; this lint does not claim
    that selected files are locked or reserved until the provider reads them.
    """
    registry_path = repo_root / ".github" / "validation-producers.json"
    if registry_path.is_symlink():
        return [f"{registry_path}: registry must not be a symlink"]
    if not registry_path.exists():
        runtime_present = (repo_root / "scripts/validation_runner.py").exists()
        wrapper_present = (repo_root / ".github/actions/validation-upload/action.yml").exists()
        if runtime_present or wrapper_present:
            return [f"{registry_path}: required when validation runner or upload wrapper is present"]
        return []  # Pre-integration checkouts keep the baseline output/retention lints.
    findings = []
    registry_file, error = _safe_repo_file(repo_root, ".github/validation-producers.json")
    if error:
        return [f"{registry_path}: {error}"]
    try:
        raw = registry_file.read_bytes()
        if len(raw) > 1024 * 1024:
            raise ValueError("registry exceeds 1 MiB budget")
        document = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_json_object)
    except (OSError, UnicodeError, ValueError, TypeError) as error:
        return [f"{registry_path}: cannot safely parse registry ({type(error).__name__})"]
    if not isinstance(document, dict) or set(document) != {"schema_version", "producers"}:
        return [f"{registry_path}: expected only schema_version and producers"]
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        findings.append(f"{registry_path}: schema_version must be integer 1")
    producers = document["producers"]
    if not isinstance(producers, list) or len(producers) > 100:
        return findings + [f"{registry_path}: producers must be a bounded array"]

    expected_ids = set(_EXPECTED_CLI_PRODUCERS) | {_EXPECTED_UPLOAD_PRODUCER["id"]}
    by_id = {}
    seen_paths = set()
    for index, row in enumerate(producers):
        if not isinstance(row, dict):
            findings.append(f"{registry_path}: producer {index + 1} must be an object")
            continue
        producer_id = row.get("id")
        if not isinstance(producer_id, str) or not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", producer_id):
            findings.append(f"{registry_path}: producer {index + 1} has an invalid id")
            continue
        if producer_id in by_id:
            findings.append(f"{registry_path}: duplicate producer id '{producer_id}'")
            continue
        by_id[producer_id] = row
        if producer_id not in expected_ids:
            findings.append(f"{registry_path}: unrecognized producer id '{producer_id}'")
            continue
        expected_path = (_EXPECTED_CLI_PRODUCERS[producer_id][0]
                         if producer_id in _EXPECTED_CLI_PRODUCERS else _EXPECTED_UPLOAD_PRODUCER["path"])
        path = row.get("path")
        if isinstance(path, str):
            _, declared_path_error = _safe_repo_file(repo_root, path)
            if declared_path_error:
                findings.append(f"{registry_path}: producer '{producer_id}' {declared_path_error}")
        if path != expected_path:
            findings.append(f"{registry_path}: producer '{producer_id}' path must be '{expected_path}'")
        if isinstance(path, str):
            if path in seen_paths:
                findings.append(f"{registry_path}: duplicate producer path '{path}'")
            seen_paths.add(path)
        if producer_id in _EXPECTED_CLI_PRODUCERS:
            output_option = _EXPECTED_CLI_PRODUCERS[producer_id][1]
            if set(row) != {"id", "path", "kind", "lifecycle", "output_options", "payload_env"}:
                findings.append(f"{registry_path}: producer '{producer_id}' has invalid CLI fields")
                continue
            if row.get("kind") != "cli" or row.get("lifecycle") != "validation-runner":
                findings.append(f"{registry_path}: producer '{producer_id}' has invalid CLI lifecycle")
            if row.get("output_options") != [output_option]:
                findings.append(f"{registry_path}: producer '{producer_id}' output_options must be [{output_option!r}]")
            if row.get("payload_env") != "VALIDATION_ARTIFACT_DIR":
                findings.append(f"{registry_path}: producer '{producer_id}' payload_env must be VALIDATION_ARTIFACT_DIR")
            source, source_error = _safe_repo_file(repo_root, expected_path)
            if source_error:
                findings.append(f"{registry_path}: producer '{producer_id}' source {source_error}")
            else:
                valid, reason = _has_managed_entrypoint(source, output_option)
                if not valid:
                    findings.append(f"{registry_path}: producer '{producer_id}' {reason}")
        else:
            expected_fields = set(_EXPECTED_UPLOAD_PRODUCER)
            if set(row) != expected_fields:
                findings.append(f"{registry_path}: upload producer has invalid fields")
            for key, value in _EXPECTED_UPLOAD_PRODUCER.items():
                if row.get(key) != value or (key in {"retention_days", "max_bytes"} and type(row.get(key)) is not int):
                    findings.append(f"{registry_path}: upload producer '{key}' must match {value!r}")
            action, action_error = _safe_repo_file(repo_root, expected_path)
            if action_error:
                findings.append(f"{registry_path}: wrapper action {action_error}")
            else:
                findings.extend(_lint_upload_action(
                    repo_root, action, _EXPECTED_UPLOAD_PRODUCER["retention_days"],
                    _EXPECTED_UPLOAD_PRODUCER["max_bytes"],
                ))
                upload_script, upload_script_error = _safe_repo_file(repo_root, "scripts/validation_upload.py")
                if upload_script_error:
                    findings.append(f"{registry_path}: upload checker {upload_script_error}")
                else:
                    findings.extend(_lint_upload_checker(upload_script, _EXPECTED_UPLOAD_PRODUCER["max_bytes"]))

    for missing in sorted(expected_ids - set(by_id)):
        findings.append(f"{registry_path}: required producer '{missing}' is missing")
    if workflow_paths is not None and _EXPECTED_UPLOAD_PRODUCER["id"] in by_id:
        findings.extend(_lint_wrapper_callers(
            repo_root, workflow_paths, _EXPECTED_UPLOAD_PRODUCER["retention_days"],
            _EXPECTED_UPLOAD_PRODUCER["max_bytes"],
        ))
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
    findings = (lint_tracked_build_outputs(repo) + lint_upload_retention(workflows, repo)
                + lint_producer_registry(repo, workflows))
    for finding in findings:
        print(finding)
    if not findings:
        print("Validation artifact lint passed")
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
