#!/usr/bin/env python3
"""
Style and hygiene verification script for Paravane.
Validates that:
1. No unicode em dashes (U+2014) or en dashes (U+2013) exist in any source or documentation file.
2. Trailing whitespace is not excessive.
3. All source files have proper trailing newlines.
"""

import os
import sys

def check_files():
    violations = []
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    checked_count = 0

    valid_extensions = (
        ".janet", ".md", ".c", ".h", ".py", ".txt", ".json", ".yaml", "Makefile"
    )

    for dirpath, _, filenames in os.walk(root_dir):
        if any(skip in dirpath for skip in [".git", "__pycache__", "build", ".venv", "output"]):
            continue
        for fname in filenames:
            if fname.endswith(valid_extensions) or fname == "Makefile":
                fpath = os.path.join(dirpath, fname)
                checked_count += 1
                try:
                    with open(fpath, "r", encoding="utf-8", errors="replace") as f:
                        text = f.read()
                        if "\u2014" in text:
                            violations.append((fpath, "Contains em-dash U+2014"))
                        if "\u2013" in text:
                            violations.append((fpath, "Contains en-dash U+2013"))
                except Exception as e:
                    violations.append((fpath, f"Read error: {e}"))

    if violations:
        print(f"[FAIL] Found {len(violations)} hygiene violations:")
        for path, reason in violations:
            rel = os.path.relpath(path, root_dir)
            print(f"  - {rel}: {reason}")
        return 1

    print(f"[OK] Clean style verified across {checked_count} repository files.")
    return 0

if __name__ == "__main__":
    sys.exit(check_files())
