
curl http://localhost:3440/callback -v -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsCenterX\":960}}"
curl http://localhost:3440/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsCenterY\":600}}"
curl http://localhost:3440/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsThreshold\":250}}"
curl http://localhost:3440/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"ToggleImOpsDetection\":true}}"

curl http://localhost:3441/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsCenterX\":960}}"
curl http://localhost:3441/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsCenterY\":600}}"
curl http://localhost:3441/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"SetImOpsThreshold\":250}}"
curl http://localhost:3441/callback -H "Content-Type: application/json" -d "{\"ToCamera\":{\"ToggleImOpsDetection\":true}}"

pause

