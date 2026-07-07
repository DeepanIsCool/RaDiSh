#!/bin/bash

# Exit on error
set -e

echo "=========================================="
echo "    Lancer AV Simulator Setup Script"
echo "=========================================="

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: python3 is not installed or not in your PATH."
    echo "Please install Python 3.10+ before running this script."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "✅ Detected Python version: $PYTHON_VERSION"

# Recreate .venv if it exists
if [ -d ".venv" ]; then
    echo "🧹 Removing existing .venv directory to avoid path conflicts..."
    rm -rf .venv
fi

# Detect uv package manager (much faster setup)
if command -v uv &> /dev/null; then
    echo "⚡ 'uv' package manager detected. Re-creating environment with uv..."
    uv venv
    source .venv/bin/activate
    uv pip install -r requirements.txt
else
    echo "📦 Creating virtual environment using standard python3 -m venv..."
    python3 -m venv .venv
    source .venv/bin/activate
    echo "📥 Installing packages (this might take a minute, e.g. PyTorch, PyQt6)..."
    pip install --upgrade pip
    pip install -r requirements.txt
fi

echo "=========================================="
echo "🎉 Setup Complete!"
echo "=========================================="
echo "To run the application, execute:"
echo "  source .venv/bin/activate"
echo "  python3 main.py"
echo "=========================================="
