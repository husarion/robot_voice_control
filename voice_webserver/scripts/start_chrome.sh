#!/bin/bash

sleep 10
PACKAGE_DIR=$(rospack find voice_webserver)
cd $PACKAGE_DIR/src
IP=$(python ./scripts/vw_config.py)
google-chrome  --user-data-dir --test-type --no-sandbox --no-default-browser-check https://$IP:3000