#!/bin/bash

set -e 

docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy