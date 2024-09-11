#!/bin/bash -x
set -o errexit

cd /home/strawlab/droneware_scripts

# curl -X POST -H 'Content-Type: application/json' -d '{"message": "hello"}' http://localhost:9500
# {"ToCamera":{"SetImOpsCenterX":960}}
# {"ToCamera":{"SetImOpsCenterY":600}}
# {"ToCamera":{"SetImOpsThreshold":250}}
# {"ToCamera":{"ToggleImOpsDetection":true}}

# to update the cookie, run this: (but substitute the new token)
# curl --cookie-jar cookie.txt http://localhost:5555/callback?token=41e39e9b-9fe2-44e9-9edb-bb1a67dcacc7  

curl --cookie cookie.txt -v -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsCenterX":960}}' http://localhost:5555/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsCenterY":600}}' http://localhost:5555/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsThreshold":250}}' http://localhost:5555/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"ToggleImOpsDetection":true}}' http://localhost:5555/callback

curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsCenterX":960}}' http://localhost:5556/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsCenterY":600}}' http://localhost:5556/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"SetImOpsThreshold":250}}' http://localhost:5556/callback
curl --cookie cookie.txt -H 'Content-Type: application/json' -d '{"ToCamera":{"ToggleImOpsDetection":true}}' http://localhost:5556/callback

