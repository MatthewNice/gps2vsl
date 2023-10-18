#!/bin/bash

echo "=========================="
echo "Starting App 'middleway' for Middleway VSL"


systemctl start gps2vsl
systemctl start rosnodeChecker
