#!/bin/bash

echo "=========================="
echo "Starting App leadAccelEstimator for {APP_PRETTY_NAME}"


systemctl start gps2vsl
systemctl start rosnodeChecker
