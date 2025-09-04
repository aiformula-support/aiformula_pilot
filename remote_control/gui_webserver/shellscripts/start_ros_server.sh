#!/bin/bash

PORT=8000

if lsof -i :$PORT; then
    echo "Port $PORT is already in use. Killing existing process..."
    kill -9 $(lsof -t -i:$PORT)
fi

# Start Python HTTP
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../webpages"
python3 -m http.server $PORT &
HTTP_SERVER_PID=$!

cleanup() {
    echo "Stopping processes..."
    kill $ROSBRIDGE_PID $HTTP_SERVER_PID 2>/dev/null
    wait $HTTP_SERVER_PID 2>/dev/null
    echo "Processes stopped."
}

trap cleanup INT TERM

wait $HTTP_SERVER_PID
