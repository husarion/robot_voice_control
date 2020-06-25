#!/bin/bash
PACKAGE_DIR=$(rospack find voice_webserver)
cd $PACKAGE_DIR/src

node server.js

