#!/bin/sh
cd ~/.node-red
npm install node-red-contrib-python3-function
npm install node-red-node-pi-gpio
npm install node-red-dashboard


mkdir %HOME/.venv
python3 -m venv %HOME/.venv  
source ~/.venv/bin/activate
pip install -r requirements.txt
