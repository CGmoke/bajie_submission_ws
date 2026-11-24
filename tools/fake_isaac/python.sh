#!/usr/bin/env bash
echo "[fake-isaac] runner called"
echo "[fake-isaac] ISAAC_PATH=$ISAAC_PATH"
printf "[fake-isaac] arg: %s\n" "$@"
echo "fake stdout"
exit 0
