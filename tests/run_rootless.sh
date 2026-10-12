#!/usr/bin/env bash
# Run a test that needs network-admin rights inside throwaway user+net+mount
# namespaces: no sudo, and the host's real networking is never touched.
#   tests/run_rootless.sh tests/static_link.sh
exec env COSIM_SANDBOX=1 unshare -Urnm "$@"
