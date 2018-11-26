## How to build and via Docker

1. instal docker (apt-get install docker)
2. docker build -t tmnt_image .
3. docker run -it --entrypoint=/bin/bash tnmt_image

Known issues ->

1. scan topic not always running
2. scan topic may not contain any data
