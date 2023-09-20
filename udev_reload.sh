#!/bin/bash

if which udevadm > /dev/null; then
  set +e # Disable exit on error
  udevadm control --reload-rules
  service udev restart
  udevadm trigger
  set -e # Re-enable exit on error
fi
