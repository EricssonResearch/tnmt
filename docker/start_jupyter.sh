#!/bin/bash

pip install jupyter ipython==5.0.0 ipykernel==4.8.0

jupyter notebook --allow-root --port 9090 --NotebookApp.token='' --ip=10.68.229.7 --no-browser
