#!/bin/sh

cleanup() {
    echo "Beende alle Prozesse..."
    echo "Killing processes $pid1, $pid2, $pid3"
    kill $pid1 $pid2 $pid3
    wait $pid1 $pid2 $pid3
    exit 0
}

trap cleanup INT

echo "Starte influx_gr6.py"
python3 influx_gr6.py &
pid1=$!
echo "influx_gr6.py läuft mit PID $pid1"

echo "Starte webapp.py"
python3 webapp.py &
pid2=$!
echo "webapp.py läuft mit PID $pid2"

echo "Starte node-red-pi"
node-red-pi --max-old-space-size=256 &
pid3=$!
echo "node-red-pi läuft mit PID $pid3"

# Warten auf Prozesse, um das Skript offen zu halten
wait $pid1 $pid2 $pid3
