#!/bin/bash

python -m pip install -r $(rospack find voice_control)/../requirements.txt --user

sudo apt install openssl

cd $(rospack find voice_webserver)/src/
! ls | grep -w 'deepspeech-0.6.0-models' && wget https://github.com/mozilla/DeepSpeech/releases/download/v0.6.0/deepspeech-0.6.0-models.tar.gz
ls | grep -w 'deepspeech-0.6.0-models.tar.gz' && tar -xzf ./deepspeech-0.6.0-models.tar.gz

yarn install 
mkdir -p cert ; cd ./cert

hostname=$1
gpu=$2
dd if=/dev/urandom of=~/.rnd bs=256 count=1
python $(rospack find voice_webserver)/src/scripts/vw_config.py --update_hostname ${hostname:-"localhost"} --gpu ${gpu:-1} 
    
openssl req -newkey rsa:4096 -x509 -sha256 -days 3650 -nodes -out cert.crt -keyout key.key -subj "/C=PL/ST=Krakow/L=Krakow/O=Husarion/OU=Tutorials/CN=localhost"
cat key.key cert.crt > server.pem
sed -i 's/ENCRYPTED/RSA/g' server.pem

