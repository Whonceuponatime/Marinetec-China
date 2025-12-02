#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d "venv" ]; then
    echo "[ERROR] Virtual environment not found. Please create it first:"
    echo "  python3 -m venv venv"
    echo "  source venv/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

if [ "$EUID" -ne 0 ]; then
    echo "[INFO] Requesting root privileges..."
    exec sudo "$0" "$@"
fi

echo "[INFO] Activating virtual environment..."
source venv/bin/activate

echo "[INFO] Starting MarineTec-China Controller..."
python3 marinetec_eicar_button.py

