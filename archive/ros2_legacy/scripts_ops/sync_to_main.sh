#!/usr/bin/env bash
# Sync changed files from QRC_demo back to the main LRC_stack repo.
# Only overwrites files that exist in both repos — never adds or deletes
# files in the main repo.
#
# Usage:
#   ./sync_to_main.sh          # preview (dry-run)
#   ./sync_to_main.sh --apply  # actually copy
set -euo pipefail

MAIN_REPO="/home/hz/COMP0225_LRC_stack"
QRC_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "$MAIN_REPO/.git" ]]; then
  echo "ERROR: main repo not found at $MAIN_REPO" >&2
  exit 1
fi

APPLY=false
if [[ "${1:-}" == "--apply" ]]; then
  APPLY=true
fi

# Get all files modified since last commit (staged + unstaged + untracked new files)
changed_files=()
while IFS= read -r f; do
  [[ -n "$f" ]] && changed_files+=("$f")
done < <(cd "$QRC_REPO" && git diff --name-only HEAD 2>/dev/null; cd "$QRC_REPO" && git diff --cached --name-only HEAD 2>/dev/null)

# Deduplicate
mapfile -t changed_files < <(printf '%s\n' "${changed_files[@]}" | sort -u)

if [[ ${#changed_files[@]} -eq 0 ]]; then
  echo "No changes to sync."
  exit 0
fi

copied=0
skipped=0
for f in "${changed_files[@]}"; do
  src="$QRC_REPO/$f"
  dst="$MAIN_REPO/$f"
  if [[ ! -f "$src" ]]; then
    continue
  fi
  if [[ ! -f "$dst" ]]; then
    echo "SKIP (not in main repo): $f"
    ((skipped++)) || true
    continue
  fi
  if $APPLY; then
    cp "$src" "$dst"
    echo "COPIED: $f"
  else
    echo "WOULD COPY: $f"
  fi
  ((copied++)) || true
done

echo "---"
if $APPLY; then
  echo "Synced $copied file(s) to $MAIN_REPO ($skipped skipped)"
  echo "Now cd $MAIN_REPO and commit."
else
  echo "$copied file(s) to sync, $skipped skipped (dry-run, use --apply to copy)"
fi
