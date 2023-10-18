#!/bin/bash

echo "=========================="
echo "Stopping App middleway"

systemctl stop rosnodeChecker
systemctl stop middleway
