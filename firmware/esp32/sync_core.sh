#!/bin/sh
# Copy the simulation-validated controller into the firmware library.
# sim_api.cpp is deliberately not copied: it is the C entry point the simulator loads, not firmware.
set -eu
cd "$(dirname "$0")"
mkdir -p lib/botcore
FILES="controller.hpp controller.cpp gains.hpp standup_policy.hpp"
for f in $FILES; do cp "../../core/$f" lib/botcore/; done
# Verify, rather than just claiming it: this script used to omit standup_policy.hpp and still print
# "updated", so the firmware could have been built against the placeholder policy while the sim ran
# the trained one. CLAUDE.md requires lib/botcore == core/; check it here so the rule has teeth.
rc=0
for f in $FILES; do
  cmp -s "../../core/$f" "lib/botcore/$f" || { echo "NOT IN SYNC: $f" >&2; rc=1; }
done
for f in lib/botcore/*; do
  b=$(basename "$f")
  case " $FILES " in *" $b "*) ;; *) echo "STALE in lib/botcore, not in core/: $b" >&2; rc=1 ;; esac
done
[ $rc -eq 0 ] || { echo "sync_core.sh FAILED" >&2; exit 1; }
echo "lib/botcore updated from core/ ($FILES) - verified identical"
