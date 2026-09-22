#!/bin/sh
# Install (symlink, or --copy) the hanan_router KLayout macros into ~/.klayout.
set -e
here=$(cd "$(dirname "$0")" && pwd)
dest="${KLAYOUT_HOME:-$HOME/.klayout}/pymacros"
mkdir -p "$dest"
for f in "$here"/pymacros/*.lym; do
  b=$(basename "$f")
  rm -f "$dest/$b"
  if [ "$1" = "--copy" ]; then cp "$f" "$dest/$b"; else ln -s "$f" "$dest/$b"; fi
  echo "installed $dest/$b"
done
