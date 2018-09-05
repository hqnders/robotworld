#!/bin/bash
cd "./src"
./robotworld -local_port=5555 -remote_ip=localhost -remote_port=5556 &
./robotworld -local_port=5556 -remote_ip=localhost -remote_port=5555
